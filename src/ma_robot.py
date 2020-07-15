#!/usr/bin/env python
from __future__ import print_function
from cmath import rect, phase
import math
import hashlib
import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from marble_artifact_detection_msgs.msg import ArtifactImg
from marble_origin_detection_msgs.msg import OriginDetectionStatus

from multi_agent import MultiAgent, ArtifactReport

# Import Ignition/Gazebo only if running in the sim so the robot doesn't need them
if rospy.get_param('multi_agent/simcomms', False):
    # Switch for ignition or gazebo here temporarily until I find a better place
    useIgnition = True

    from gazebo_msgs.msg import ModelState
    if useIgnition:
        from subt_msgs.srv import SetPose
    else:
        from gazebo_msgs.srv import SetModelState


def getDist(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)


def getDist2D(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)


def getYaw(orientation):
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    return math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z))


def averagePose(history):
    pos = Point()
    yaw = 0
    for pose in history:
        pos.x = pos.x + pose.position.x
        pos.y = pos.y + pose.position.y
        pos.z = pos.z + pose.position.z
        yaw = yaw + rect(1, getYaw(pose.orientation))

    pos.x = pos.x / float(len(history))
    pos.y = pos.y / float(len(history))
    pos.z = pos.z / float(len(history))
    yaw = math.degrees(phase(yaw))

    return pos, yaw


def angleDiff(a, b):
    # Computes a-b, preserving the correct sign (counter-clockwise positive angles)
    # All angles are in degrees
    a = (360000 + a) % 360
    b = (360000 + b) % 360
    d = a - b
    d = (d + 180) % 360 - 180
    return d


class MARobot(MultiAgent):
    """ Initialize a multi-agent robot node """

    def __init__(self):
        # Get all of the parent class variables
        MultiAgent.__init__(self)

        # Distance to maintain goal point deconfliction
        self.deconflictRadius = rospy.get_param('multi_agent/deconflictRadius', 2.5)
        # Time stopped to report if stuck
        self.stopCheck = rospy.get_param('multi_agent/stopCheck', 30)
        # Distance from Anchor to drop beacons automatically
        self.maxAnchorDist = rospy.get_param('multi_agent/anchorDropDist', 100)
        # Distance to drop beacons automatically
        self.maxDist = rospy.get_param('multi_agent/dropDist', 30)
        # Minimum distance between junctions before dropping another beacon
        self.junctionDist = rospy.get_param('multi_agent/junctionDist', 10)
        # Whether to use turn detection to drop beacons
        self.turnDetect = rospy.get_param('multi_agent/turnDetect', True)
        # Whether this agent should delay their drop so the trailing robot can
        self.delayDrop = rospy.get_param('multi_agent/delayDrop', False)
        # Whether to backtrack to deploy a beacon
        self.reverseDrop = rospy.get_param('multi_agent/reverseDrop', False)
        # Topics for publishers
        homeTopic = rospy.get_param('multi_agent/homeTopic', 'report_artifact')
        stopTopic = rospy.get_param('multi_agent/stopTopic', 'nearness_controller/enable_control')
        waitTopic = rospy.get_param('multi_agent/waitTopic', 'origin_detection_status')
        baseCommTopic = rospy.get_param('multi_agent/baseCommTopic', 'base_comm')
        goalTopic = rospy.get_param('multi_agent/goalTopic', 'ma_goal')
        pathTopic = rospy.get_param('multi_agent/pathTopic', 'ma_goal_path')

        # Static anchor position
        self.anchorPos = Point()
        self.anchorPos.x = rospy.get_param('multi_agent/anchorX', 1.0)
        self.anchorPos.y = rospy.get_param('multi_agent/anchorY', 0.0)
        self.anchorPos.z = rospy.get_param('multi_agent/anchorZ', 0.1)

        self.startedMission = False
        self.initialPose = False
        self.history = []
        self.hislen = self.rate * 10  # How long the odometry history should be
        self.minAnchorDist = 10  # Minimum distance before a beacon is ever dropped
        self.report = False
        self.newTask = False
        self.newTaskExternal = False
        self.taskCount = 0
        self.beaconCommLost = 0
        self.stopStart = True
        self.mode = 'Explore'
        self.stuck = 0

        self.task_pub = rospy.Publisher('task', String, queue_size=10)
        self.deploy_pub = rospy.Publisher('deploy', Bool, queue_size=10)
        self.reset_pub = rospy.Publisher('reset_artifacts', Bool, queue_size=10)
        self.num_pub = rospy.Publisher('num_neighbors', Int8, queue_size=10)
        self.home_pub = rospy.Publisher(homeTopic, Bool, queue_size=10)
        self.stop_pub = rospy.Publisher(stopTopic, Bool, queue_size=10)
        self.comm_pub = rospy.Publisher(baseCommTopic, Bool, queue_size=10)
        self.goal_pub = rospy.Publisher(goalTopic, PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher(pathTopic, Path, queue_size=10)
        self.wait_sub = rospy.Subscriber(waitTopic, OriginDetectionStatus, self.WaitMonitor)
        self.task_sub = rospy.Subscriber('task', String, self.TaskMonitor)

        self.pub_guiTask = {}
        self.pub_guiTask['estop'] = rospy.Publisher('estop', Bool, queue_size=10)
        self.pub_guiTask['estop_cmd'] = rospy.Publisher('estop_cmd', Bool, queue_size=10)
        self.pub_guiTask['radio_reset_cmd'] = rospy.Publisher('radio_reset_cmd', Bool, queue_size=10)
        self.pub_guiGoal = rospy.Publisher('guiGoalPoint', PoseStamped, queue_size=10)

        self.commListen = True

    def WaitMonitor(self, data):
        if data.status > 0:
            self.wait = False

    def TaskMonitor(self, data):
        normalStatus = ['Able to plan home', 'Exploring']
        if data.data != self.agent.status and data.data not in normalStatus:
            self.newTask = data.data
            self.newTaskExternal = True
        elif self.taskCount > 10:
            self.newTask = False
            self.newTaskExternal = False
            self.taskCount = 0
        else:
            self.taskCount += 1

    def publishGUITask(self):
        if self.agent.guiTaskValue == 'True':
            data = True
        elif self.agent.guiTaskValue == 'False':
            data = False
        else:
            data = self.agent.guiTaskValue

        self.pub_guiTask[self.agent.guiTaskName].publish(data)

    def updateHistory(self):
        # Check whether we've started the mission by moving 5 meters
        if not self.initialPose:
            self.initialPose = self.agent.odometry.pose.pose
        elif not self.startedMission:
            # Do this here so we only do this calculation until leaving the starting area
            if getDist(self.agent.odometry.pose.pose.position, self.initialPose.position) > 5:
                self.startedMission = True

        self.history.append(self.agent.odometry.pose.pose)
        if len(self.history) > self.hislen:
            self.history = self.history[-self.hislen:]

    def updateStatus(self, status):
        if self.newTask and status not in self.newTask:
            self.newTask += '+++' + status
        else:
            self.newTask = status
            self.newTaskExternal = False

    def getStatus(self):
        if self.newTask:
            return self.agent.status + '+++' + self.newTask
        else:
            return self.agent.status

    def deployBeacon(self, inplace, dropReason):
        deploy = False
        # Find the first available beacon for this agent
        for beacon in self.beacons.values():
            if beacon.owner and not beacon.active:
                deploy = beacon.id
                break

        if deploy:
            # Stop the robot and publish message to deployment mechanism
            self.stop()
            pose = self.agent.odometry.pose.pose

            # TODO doesn't currently work since the node is paused!
            self.agent.status = 'Deploy'
            self.task_pub.publish(self.agent.status)

            if self.useSimComms:
                # Either need to identify location before comm loss, or make this a guidance
                # command to return to a point in range
                if useIgnition:
                    service = '/subt/set_pose'
                    rospy.wait_for_service(service)
                    set_state = rospy.ServiceProxy(service, SetPose)
                else:
                    service = '/gazebo/set_model_state'
                    rospy.wait_for_service(service)
                    set_state = rospy.ServiceProxy(service, SetModelState)

                # Get the yaw from the quaternion
                yaw = getYaw(pose.orientation)

                if inplace:
                    offset = 0.5
                else:
                    offset = 6

                pose.position.x = pose.position.x - math.cos(yaw) * offset
                pose.position.y = pose.position.y - math.sin(yaw) * offset

                state = ModelState()
                state.model_name = deploy
                state.pose = pose

            print(self.id, 'deploying beacon', deploy, 'for', dropReason)
            try:
                if self.useSimComms:
                    # Drop the simulated beacon, and pause to simulate drop
                    if useIgnition:
                        ret = set_state(deploy, pose)
                        rospy.sleep(3)
                        print(ret.success)
                    else:
                        ret = set_state(state)
                        rospy.sleep(3)
                        print(ret.status_message)
                else:
                    # Wait to stop, send deploy message, then wait for deployment to finish
                    rospy.sleep(3)
                    self.deploy_pub.publish(True)
                    rospy.sleep(10)

                # Resume the mission
                self.stop_pub.publish(True)
                self.deconflictExplore()
                self.deploy_pub.publish(False)

                self.numBeacons = self.numBeacons - 1
                self.beacons[deploy].active = True
                self.beacons[deploy].simcomm = True
                self.beacons[deploy].pos = pose.position
            except Exception as e:
                rospy.logerr('Error deploying beacon %s', str(e))
        else:
            print(self.id, "no beacon to deploy")
            # Most likely reason it thought we had a beacon is due to restart.  So set num=0.
            self.numBeacons = 0
            self.mode = 'Explore'

    def beaconCheck(self):
        # Check if we need to drop a beacon if we have any beacons to drop
        if self.numBeacons > 0:
            pose = self.agent.odometry.pose.pose
            numBeacons = 0
            numDistBeacons = 0
            dropBeacon = False
            dropReason = ''

            # We're connected to the mesh, either through anchor or beacon(s)
            if self.base.incomm:
                self.beaconCommLost = 0

                # Once we pass the maxDist we could set a flag so we don't keep recalculating this
                anchorDist = getDist(pose.position, self.anchorPos)
                # Beacon distance based drop only kicks in once out of anchor range
                checkDist = self.maxAnchorDist

                # If we're too close (like for the initial node drop), never drop a beacon
                if anchorDist < self.minAnchorDist:
                    return

                # If we're at end of anchor range, drop beacon
                if anchorDist > self.maxAnchorDist and not self.delayDrop:
                    dropBeacon = True
                    dropReason = 'anchor distance'
                    checkDist = self.maxDist

                # Always drop a beacon if we're at a node and we're in comm
                # If beacons are strong enough may want to restrict distance
                if self.agent.atnode.data:
                    dropBeacon = True
                    dropReason = 'at junction'
                    checkDist = self.junctionDist
                # Check if it looks like we're going around a corner
                elif self.turnDetect and len(self.history) == self.hislen:
                    pos1, yaw1 = averagePose(self.history[:int(0.4 * self.hislen)])
                    pos2, yaw2 = averagePose(self.history[int(0.6 * self.hislen):])

                    # Check that we've turned and moved far enough, over history and last second
                    # Will need to retune these for real vehicle dynamics
                    if (getDist(pos1, pos2) > 4 and abs(angleDiff(yaw1, yaw2)) > 30 and
                            getDist(self.history[-2].position, self.history[-1].position) > 0.5):
                        dropBeacon = True
                        dropReason = 'at turn'
                        checkDist = self.junctionDist + 5

                for beacon in self.beacons.values():
                    if beacon.active:
                        # Count the beacons we know about, and check distance
                        numBeacons = numBeacons + 1
                        dist = getDist(pose.position, beacon.pos)

                        # Count how many beacons are past max range
                        if dist > checkDist:
                            numDistBeacons = numDistBeacons + 1
                            dropBeacon = True
                            if dropReason == '' or dropReason == 'anchor distance':
                                dropReason = 'beacon distance'

                # Cancel the drop if we have more than one beacon in range
                if numBeacons - numDistBeacons > 0:
                    dropBeacon = False

                # Prevent dropping after returning home after the first beacon drop
                # TODO look at the angle between anchor and first beacon and calculate positions
                # This only works for straight departure now
                if numBeacons > 0 and anchorDist < self.maxAnchorDist and pose.position.y < 1 and pose.position.y > 1:
                    dropBeacon = False

                if dropBeacon:
                    if self.delayDrop:
                        self.delayDrop = False
                    else:
                        self.deployBeacon(True, dropReason)
            elif self.reverseDrop:
                self.beaconCommLost += 1
                # If we're not talking to the base station, attempt to reverse drop
                if self.beaconCommLost > 5:
                    self.mode = 'Deploy'
                    self.beaconCommLost = 0

    def artifactCheck(self, agent):
        # Check the artifact list received from the artifact manager for new artifacts
        for artifact in agent.newArtifacts.artifacts:
            artifact_id = (str(artifact.position.x) +
                           str(artifact.position.y) +
                           str(artifact.position.z))
            if artifact_id not in self.artifacts:
                # Check if there's a similar artifact already stored so we don't report
                if agent.id == self.id:
                    ignore = False
                    for artifact2 in self.artifacts.values():
                        if getDist2D(artifact.position, artifact2.artifact.position) < 3:
                            ignore = True

                    if not ignore:
                        self.report = True

                # Now add the artifact to the array
                self.artifacts[artifact_id] = ArtifactReport(agent.id, artifact, artifact_id)
                print(self.id, 'new artifact from', agent.id, artifact.obj_class, artifact_id)

    def artifactCheckImages(self, agent):
        image_id = agent.newArtifactImages.image_id
        if image_id:
            for artifact in self.artifacts.values():
                if image_id == artifact.artifact.image_id:
                    print(self.id, 'adding image', image_id, 'to', artifact.id)
                    artifact.image = agent.newArtifactImages
                    agent.newArtifactImages = ArtifactImg()
                    break

    def artifactCheckReport(self):
        # If we didn't add anything new, check if any still need reported
        if not self.report:
            for artifact in self.artifacts.values():
                if artifact.agent_id == self.id and not artifact.reported:
                    self.report = True
                    break

        # Identify our report so we can track that the base station has seen it
        if self.report:
            print('will report...')
            artifactString = repr(self.agent.newArtifacts.artifacts).encode('utf-8')
            self.agent.lastArtifact = hashlib.md5(artifactString).hexdigest()

    def deconflictGoals(self):
        # Get all of the goals into a list
        goals = self.agent.goals.goals

        if not goals:
            # Set the goal to the frontier goal
            self.agent.goal.pose = self.agent.exploreGoal
            self.agent.goal.path = self.agent.explorePath
        elif len(goals) == 1:
            # If we only have one potential goal, just go there
            # May want to consider stopping in place if there is a conflict!
            self.agent.goal = goals[0]
        else:
            # TODO add check for location of neighbor and don't go there
            # Otherwise, deconflict with neighbor goals
            # Assume conflict to start
            conflict = True
            i = 0
            while conflict and i < len(goals):
                # Check each goal in order for conflict with any neighbors
                gpos = goals[i].pose.pose.position
                for neighbor in self.neighbors.values():
                    npos = neighbor.goal.pose.pose.position
                    # Check whether they are within the defined range
                    if getDist(gpos, npos) < self.deconflictRadius:
                        # If our cost is more than the neighbor, don't go to this goal
                        if goals[i].cost.data > neighbor.goal.cost.data:
                            conflict = True
                            if not self.newTask:
                                self.updateStatus('Replanning')
                            print(str(self.id) + " replanning")
                            # Don't need to check any more neighbors for this goal if conflict
                            break
                        else:
                            conflict = False

                    # No conflict with this neighbor
                    # If each neighbor has no conflict with this goal, the while loop ends
                    conflict = False

                # Check the next goal
                i += 1

            # Set the goal to the last goal without conflict
            self.agent.goal = goals[i - 1]

    def stop(self):
        # Stop the robot by publishing no path, but don't change the displayed goal
        self.agent.status = 'Stop'
        self.task_pub.publish(self.agent.status)
        self.stop_pub.publish(False)

        if self.stopStart and self.useSimComms:
            print(self.id, "stopping")
            path = Path()
            path.header.frame_id = 'world'
            inplace = PoseStamped()
            inplace.pose = self.agent.odometry.pose.pose
            inplace.header.frame_id = 'world'
            path.poses.append(inplace)
            path.poses.append(inplace)
            self.goal_pub.publish(inplace)
            self.path_pub.publish(path)
            self.stopStart = False

    def setGoalPoint(self, reason):
        # Set the goal point for frontier exploration
        if reason == 'guiCommand':
            self.pub_guiGoal.publish(self.agent.guiGoalPoint)
        else:
            # For now Home and Report need this set to switch frontier exploration
            self.home_pub.publish(True)

        # Set the new task, and use frontier exploration's goal and path
        self.stopStart = True
        self.agent.status = reason
        self.task_pub.publish(self.agent.status)
        self.goal_pub.publish(self.agent.exploreGoal)
        self.path_pub.publish(self.agent.explorePath)
        self.agent.goal.pose = self.agent.exploreGoal
        self.agent.goal.path = self.agent.explorePath

    def deconflictExplore(self):
        # Explore with goal deconfliction
        self.stopStart = True
        self.agent.status = 'Explore'
        self.home_pub.publish(False)
        self.task_pub.publish(self.agent.status)
        # Find the best goal point to go to and publish
        self.deconflictGoals()
        self.goal_pub.publish(self.agent.goal.pose)
        self.path_pub.publish(self.agent.goal.path)

    def run(self):
        # Update our comm status for anyone who needs it
        self.comm_pub.publish(self.base.incomm)

        # Update movement history
        self.updateHistory()

        # Do status checks once we've started the mission
        if self.startedMission and self.agent.status != 'Stop':
            if self.agent.goal.path.poses and len(self.history) == self.hislen:
                # Check if we've been stopped if we have a goal
                if getDist(self.history[0].position, self.history[-1].position) < 0.5:
                    self.stuck += 1
                else:
                    self.stuck = 0

                if self.stuck > self.stopCheck:
                    self.updateStatus('Stuck')
                    print(self.id, 'has not moved!')
            elif not self.agent.goal.path.poses:
                # Report no path available
                self.updateStatus('No Path')
                # print(self.id, 'no path!')

        # For future use, to handle tasks coming from the topic
        # Maybe this should just be separate topic
        if self.newTask and self.newTaskExternal:
            print(self.id, 'received external new task', self.newTask)

        checkBeacon = True
        # Manage the newest task sent
        if self.agent.guiAccept:
            if self.agent.guiTaskName == 'task':
                if self.agent.guiTaskValue == 'Explore':
                    # Overrides goal point to return to explore
                    self.agent.guiGoalAccept = False
                    self.mode = 'Explore'
                elif self.agent.guiTaskValue == 'Home':
                    self.mode = 'Home'
                elif self.agent.guiTaskValue == 'Stop':
                    self.mode = 'Stop'
                elif self.agent.guiTaskValue == 'Deploy':
                    self.deployBeacon(True, 'GUI Command')
                    checkBeacon = False
                    self.mode = 'Explore'

                # Disable the estop.  'Stop' will re-enable it
                self.stop_pub.publish(True)
            else:
                self.publishGUITask()

            self.agent.guiAccept = False

        # Check whether to drop a beacon, as long as we weren't commanded by the GUI
        if checkBeacon:
            self.beaconCheck()

        num_neighbors = 0
        # Time check for "current" neighbors.  Make sure we don't have negative time.
        if rospy.get_rostime() > rospy.Time(0) + self.commThreshold * 30:
            neighbor_check = rospy.get_rostime() - self.commThreshold * 30
        else:
            neighbor_check = rospy.get_rostime()

        # Get our neighbors' artifacts so we can deconflict reporting
        for neighbor in self.neighbors.values():
            self.artifactCheck(neighbor)

            # Count how many neighbors we have current goal information for deconfliction
            # Using 60 seconds for now
            if neighbor.lastMessage > neighbor_check:
                num_neighbors += 1

        # Publish the number of neighbors that frontier exploration should consider
        self.num_pub.publish(num_neighbors)

        # Make sure our internal artifact list is up to date, and if we need to report
        self.artifactCheck(self.agent)
        self.artifactCheckImages(self.agent)
        self.artifactCheckReport()

        # Decide which goal to go to based on status in this precedence:
        # Report Artifacts
        # GUI Return Home
        # GUI Stop
        # GUI Goal Point
        # Explore
        if self.report:
            # Once we see the base has our latest artifact report we can stop going home
            if self.solo or self.base.lastArtifact == self.agent.lastArtifact:
                # Turn off reporting
                self.report = False
                for artifact in self.artifacts.values():
                    artifact.reported = True

                # Resume normal operation (check mode or explore)
                print(self.id, 'resuming operation...')
            else:
                if self.agent.status != 'Report':
                    print(self.id, 'return to report...')
                self.setGoalPoint('Report')
                # Don't check for other mode when in report
                return True

        # Check for a mode from GUI or MA, or explore normally
        if self.mode == 'Home':
            self.setGoalPoint('Home')
        elif self.mode == 'Stop':
            self.stop()
        elif self.mode == 'Deploy':
            print(self.id, 'reverse deploy mode')
            if self.base.incomm:
                self.deployBeacon(True, 'Regain comms')
                self.mode = 'Explore'
            else:
                if not self.newTask:
                    self.updateStatus('Regain comms deploy')
                self.setGoalPoint('Home')
        elif self.agent.guiGoalAccept:
            if (getDist(self.agent.odometry.pose.pose.position,
                        self.agent.guiGoalPoint.pose.position) < 1.0):
                print(self.id, 'resuming exploration...')
                self.agent.guiGoalAccept = False
                # May want to add other options for tasks when it reaches the goal
                self.deconflictExplore()
            else:
                if self.agent.status != 'guiCommand':
                    print(self.id, 'setting GUI Goal Point...')
                self.setGoalPoint('guiCommand')
        else:
            # Normal exploration with coordination
            self.deconflictExplore()

        return True
