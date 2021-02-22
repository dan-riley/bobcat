#!/usr/bin/env python
from __future__ import print_function
import math
import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from marble_origin_detection_msgs.msg import OriginDetectionStatus
from bobcat.msg import AgentReset

from util.helpers import getDist, getDist2D, getYaw, averagePose, averagePosition, angleDiff
from BOBCAT import BOBCAT


class BCRobot(BOBCAT):
    """ Initialize a multi-agent robot node """

    def __init__(self):
        # Get all of the parent class variables
        BOBCAT.__init__(self)

        # Distance to maintain goal point deconfliction
        self.deconflictRadius = rospy.get_param('bobcat/deconflictRadius', 2.5)
        # Time stopped to report if stuck
        self.stopCheck = rospy.get_param('bobcat/stopCheck', 30)
        # Distance from Anchor to drop beacons automatically
        self.maxAnchorDist = rospy.get_param('bobcat/anchorDropDist', 100)
        # Distance to drop beacons automatically
        self.maxDist = rospy.get_param('bobcat/dropDist', 30)
        # Minimum distance between junctions before dropping another beacon
        self.junctionDist = rospy.get_param('bobcat/junctionDist', 10)
        # Whether to use turn detection to drop beacons
        self.turnDetect = rospy.get_param('bobcat/turnDetect', True)
        # Whether this agent should delay their drop so the trailing robot can
        self.delayDrop = rospy.get_param('bobcat/delayDrop', False)
        # Whether to backtrack to deploy a beacon
        self.reverseDropEnable = rospy.get_param('bobcat/reverseDrop', False)
        self.reverseDrop = self.reverseDropEnable
        # Topics for publishers
        homeTopic = rospy.get_param('bobcat/homeTopic', 'report_artifact')
        stopTopic = rospy.get_param('bobcat/stopTopic', 'nearness_controller/enable_control')
        waitTopic = rospy.get_param('bobcat/waitTopic', 'origin_detection_status')
        baseCommTopic = rospy.get_param('bobcat/baseCommTopic', 'base_comm')
        goalTopic = rospy.get_param('bobcat/goalTopic', 'ma_goal')
        pathTopic = rospy.get_param('bobcat/pathTopic', 'ma_goal_path')

        # Static anchor position
        self.anchorPos = Point()
        self.anchorPos.x = rospy.get_param('bobcat/anchorX', 1.0)
        self.anchorPos.y = rospy.get_param('bobcat/anchorY', 0.0)
        self.anchorPos.z = rospy.get_param('bobcat/anchorZ', 0.1)

        self.startedMission = False
        self.initialPose = False
        self.bl_beacons = []
        self.history = []
        self.hislen = self.rate * 10  # How long the odometry history should be
        self.minAnchorDist = 10  # Minimum distance before a beacon is ever dropped
        self.report = False
        self.newStatus = False
        self.statusCount = 0
        self.beaconCommLost = 0
        self.baseRegain = 0
        self.stopStart = True
        self.mode = 'Explore'
        self.stuck = 0
        self.planner_status = True
        self.launch_status = True
        self.isAerial = False
        self.useTraj = False

        self.task_pub = rospy.Publisher('task', String, queue_size=10)
        self.deploy_pub = rospy.Publisher('deploy', Bool, queue_size=10)
        self.deploy_breadcrumb_pub = rospy.Publisher('breadcrumb/deploy', Empty, queue_size=10)
        self.reset_pub = rospy.Publisher('reset_artifacts', Bool, queue_size=10)
        self.num_pub = rospy.Publisher('num_neighbors', Int8, queue_size=10)
        self.traj_pub = rospy.Publisher('follow_traj', Bool, queue_size=10)
        self.home_pub = rospy.Publisher(homeTopic, Bool, queue_size=10)
        self.stop_pub = rospy.Publisher(stopTopic, Bool, queue_size=10)
        self.comm_pub = rospy.Publisher(baseCommTopic, Bool, queue_size=10)
        self.goal_pub = rospy.Publisher(goalTopic, PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher(pathTopic, Path, queue_size=10)
        self.wait_sub = rospy.Subscriber(waitTopic, OriginDetectionStatus, self.WaitMonitor)
        self.planner_sub = rospy.Subscriber('planner_status', Bool, self.PlannerMonitor)
        self.launch_sub = rospy.Subscriber('velocity_controller/enable', Bool, self.LaunchMonitor)

        self.pub_guiTask = {}
        self.pub_guiTask['estop'] = rospy.Publisher('estop', Bool, queue_size=10)
        self.pub_guiTask['estop_cmd'] = rospy.Publisher('estop_cmd', Bool, queue_size=10)
        self.pub_guiTask['radio_reset_cmd'] = rospy.Publisher('radio_reset_cmd', Bool, queue_size=10)
        self.pub_guiGoal = rospy.Publisher('guiGoalPoint', PoseStamped, queue_size=10)

        # Create sphere markers for blacklist points, at the same size as the blacklist
        self.pub_blacklist = rospy.Publisher('blacklist', Marker, queue_size=10, latch=True)
        self.blgoals = []
        self.blacklist = Marker()
        self.blacklist.header.frame_id = 'world'
        self.blacklist.type = self.blacklist.SPHERE_LIST
        self.blacklist.action = self.blacklist.ADD
        self.blacklist.scale.x = self.deconflictRadius * 2
        self.blacklist.scale.y = self.deconflictRadius * 2
        self.blacklist.scale.z = self.deconflictRadius * 2
        self.blacklist.color.a = 1.0
        self.blacklist.color.r = 1.0
        self.blacklist.color.g = 0.0
        self.blacklist.color.b = 1.0

        self.commListen = True

    def WaitMonitor(self, data):
        if data.status > 0:
            self.wait = False

    def PlannerMonitor(self, data):
        self.planner_status = data.data
        if not self.planner_status:
            self.updateStatus('Unable to plan')

    def LaunchMonitor(self, data):
        self.isAerial = True
        self.launch_status = data.data

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
        if self.newStatus and status not in self.newStatus:
            self.newStatus += '+++' + status
        else:
            self.newStatus = status

    def getStatus(self):
        # Clear out the additional status messages every so often
        if self.statusCount > 5:
            self.newStatus = False
            self.statusCount = 0
        else:
            self.statusCount += 1

        if self.newStatus:
            status = self.agent.status + '+++' + self.newStatus
        else:
            status = self.agent.status

        return status

    def beaconDistCheck(self, pose, checkDist, dropBeacon):
        numBeacons = 0
        numDistBeacons = 0
        for beacon in self.beacons.values():
            if beacon.active:
                # Count the beacons we know about, and check distance
                numBeacons = numBeacons + 1
                dist = getDist(pose.position, beacon.pos)

                # Count how many beacons are past max range
                if dist > checkDist:
                    numDistBeacons = numDistBeacons + 1
                    dropBeacon = True

        # Cancel the drop if we have more than one beacon in range
        if numBeacons - numDistBeacons > 0:
            dropBeacon = False

        return dropBeacon, numBeacons, numDistBeacons

    def beaconCheck(self):
        # Check if we need to drop a beacon if we have any beacons to drop
        if self.numBeacons > 0:
            pose = self.agent.odometry.pose.pose
            dropBeacon = False
            dropReason = ''

            # We're connected to the mesh, either through anchor or beacon(s)
            if self.base.incomm:
                self.beaconCommLost = 0
                # Reset to global config so we can manipulate after cancelled drops
                # This should re-enable dropping if we re-gain comms after a cancelled drop
                self.reverseDrop = self.reverseDropEnable

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

                dropBeacon, numBeacons, numDistBeacons = self.beaconDistCheck(pose, checkDist, dropBeacon)

                if numDistBeacons > 0 and (dropReason == '' or dropReason == 'anchor distance'):
                    dropReason = 'beacon distance'

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
                    # Check if we've already attempted a reverse drop in this area
                    # Sometimes there are deadzones and this can cause a loop if not accounted for
                    for bl in self.bl_beacons:
                        if getDist(pose.position, bl) < self.junctionDist:
                            self.reverseDrop = False
                            rospy.loginfo(self.id + ' skipping reverse drop due to previous try')
                            break

                    # Change the mode to deploy, and add to the list of previously tried positions
                    if self.reverseDrop:
                        self.mode = 'Deploy'
                        self.bl_beacons.append(pose.position)

                    # Reset the counter so we don't attempt again right away
                    self.beaconCommLost = 0

    def artifactCheckReport(self):
        # If we didn't add anything new, check if any still need reported
        if not self.report:
            for artifact in self.artifacts.values():
                if artifact.agent_id == self.id and not artifact.reported:
                    self.report = True
                    break

        # Identify our report so we can track that the base station has seen it
        if self.report:
            rospy.loginfo('will report...')

    def checkBlacklist(self, goal):
        addBlacklist = True
        # Make sure it's not already in a blacklist radius
        for bgoal in self.blacklist.points:
            if getDist(bgoal, goal) < self.deconflictRadius:
                addBlacklist = False

        return addBlacklist

    def addBlacklist(self, goal):
        if self.checkBlacklist(goal):
            goalstr = str(goal.x) + '-' + str(goal.y) + '-' + str(goal.z)
            rospy.loginfo(self.id + ' added ' + goalstr + ' to blacklist')
            self.blacklist.points.append(goal)
            self.pub_blacklist.publish(self.blacklist)

    def deconflictGoals(self):
        # Get all of the goals into a list
        goals = self.agent.goals.goals
        # Last goal and path chosen
        curgoal = self.agent.goal.pose.pose.position
        curpos = self.agent.odometry.pose.pose.position
        # Keep going to long distance goal, unless it's home, or we get a new potential goal close
        # TODO make these ranges based off of sensor range in launch file
        if (getDist(curpos, self.agent.exploreGoal.pose.position) > 5.0 and
                getDist(curpos, curgoal) > 7.0 and
                not (curgoal.x == 0 and curgoal.y == 0 and curgoal.z == 0)):

            # Make sure this goal isn't blacklisted, which could happen after it was chosen
            conflict = False
            for goal in self.blacklist.points:
                if getDist(curgoal, goal) < self.deconflictRadius:
                    conflict = True
                    break

            if not conflict:
                # Check if there's an updated path for this goal, or one near it
                if not goals:
                    if getDist(curgoal, self.agent.exploreGoal.pose.position) < 0.5:
                        self.agent.goal.pose = self.agent.exploreGoal
                        self.agent.goal.path = self.agent.explorePath
                else:
                    for goal in goals:
                        if getDist(curgoal, goal.pose.pose.position) < 0.5:
                            self.agent.goal = goal
                            break

                rospy.loginfo(self.id + ' continuing to long distance goal')
                return

        if not goals:
            # Set the goal to the frontier goal
            # This should cause us to navigate to the goal, then go home if still no plan
            self.agent.goal.pose = self.agent.exploreGoal
            self.agent.goal.path = self.agent.explorePath
        elif len(goals) == 1:
            # If we only have one potential goal, just go there
            # May want to consider stopping in place if there is a conflict!
            self.agent.goal = goals[0]
        else:
            # TODO add check for location of neighbor and don't go there
            # Otherwise, deconflict with neighbor goals
            # Start true to initiate loop
            conflict = True
            i = 0
            goodGoal = None
            while conflict and i < len(goals):
                # Check each goal in order for conflict with any neighbors
                gpos = goals[i].pose.pose.position
                # Assume the point will be good to start
                conflict = False

                # Global planner should take care of this now, but it's a double check
                for goal in self.blacklist.points:
                    if getDist(gpos, goal) < self.deconflictRadius:
                        conflict = True
                        self.updateStatus('Replanning Blacklist')
                        rospy.loginfo(self.id + ' replanning due to blacklist ' + str(i))
                        break

                if not conflict:
                    # If it's not blacklisted, identify the 'best' (first) goal
                    if not goodGoal:
                        goodGoal = goals[i]
                    # Check each neighbors' goal for conflict
                    for neighbor in self.neighbors.values():
                        npos = neighbor.goal.pose.pose.position
                        # Check whether they are within the defined range
                        if getDist(gpos, npos) < self.deconflictRadius:
                            # If our cost is more than the neighbor, don't go to this goal
                            if goals[i].cost.data > neighbor.goal.cost.data:
                                conflict = True
                                self.updateStatus('Replanning Neighbor')
                                rospy.loginfo(self.id + ' replanning due to neighbor ' + str(i))
                                # Don't need to check any more neighbors for this goal if conflict
                                break

                # Check the next goal
                i += 1

            # Decide which goal to use, or whether to go home
            self.useTraj = False
            if conflict:
                # Conflict will still be true if all goals conflict, so use the best
                if goodGoal:
                    self.agent.goal = goodGoal
                    if len(goals) > 1:
                        rospy.loginfo(self.id + ' all goals conflict, using best')
                else:
                    # All goals blacklisted, start going home, but clear the blacklist
                    self.agent.goal.pose = self.agent.exploreGoal
                    self.agent.goal.path = self.agent.explorePath
                    self.blacklist.points = []
                    rospy.loginfo(self.id + ' all goals blacklisted ' + str(len(goals)))
                    self.useTraj = True
            else:
                # Set the goal to the last goal without conflict
                self.agent.goal = goals[i - 1]

        # Final blacklist check for single goal
        if not goals or len(goals) == 1:
            goal = self.agent.goal.pose.pose.position
            if len(self.agent.goal.path.poses) > 0:
                pathend = self.agent.goal.path.poses[-1].pose.position
            else:
                pathend = goal

            if not self.checkBlacklist(goal) or not self.checkBlacklist(pathend):
                rospy.loginfo(self.id + ' only goal is blacklisted, using trajectory follower')
                self.useTraj = True

    def stop(self):
        # Stop the robot by publishing no path, but don't change the displayed goal
        self.agent.status = 'Stop'
        self.task_pub.publish(self.agent.status)
        self.stop_pub.publish(False)

        if self.stopStart and self.useSimComms:
            rospy.loginfo(self.id + ' stopping')
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
            # Publishing resets the seq, but we're using that to track new commands, so save it
            seq = self.agent.guiGoalPoint.header.seq
            self.pub_guiGoal.publish(self.agent.guiGoalPoint)
            self.agent.guiGoalPoint.header.seq = seq
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

        if not self.planner_status and reason != 'guiCommand':
            self.traj_pub.publish(True)
            self.updateStatus('Following Trajectory')
            rospy.loginfo(self.id + ' using trajectory follower for home')
        else:
            self.traj_pub.publish(False)

    def run(self):
        # Update our comm status for anyone who needs it
        self.comm_pub.publish(self.base.incomm)

        # Update movement history
        self.updateHistory()

        # Do status checks once we've started the mission
        if self.startedMission and self.agent.status != 'Stop' and 'A' not in self.id:
            if self.agent.goal.path.poses and len(self.history) == self.hislen:
                # Check if we've been stopped if we have a goal
                if (getDist(self.history[0].position, self.history[-1].position) < 0.5 and
                        abs(angleDiff(math.degrees(getYaw(self.history[0].orientation)),
                                      math.degrees(getYaw(self.history[-1].orientation)))) < 60):
                    self.stuck += 1
                    # Add the current goal to potential blacklist points
                    self.blgoals.append(self.agent.goal.pose.pose.position)
                else:
                    self.stuck = 0
                    self.blgoals = []

                # If stuck, report and append to blacklist so we don't try to go here again
                if self.stuck >= self.stopCheck:
                    # Only add to the blacklist at the stopCheck intervals,
                    # or else they get added too often
                    if self.stuck % self.stopCheck == 0:
                        # Get the average goal position
                        avgGoal = averagePosition(self.blgoals)
                        # Remove outliers
                        newgoals = []
                        for goal in self.blgoals:
                            if getDist(goal, avgGoal) < self.deconflictRadius * 2:
                                newgoals.append(goal)

                        # Get the new average goal position, and only if there are enough
                        if len(newgoals) > self.hislen / 2:
                            avgGoal = averagePosition(newgoals)

                            # Make sure it's not the origin
                            if not (avgGoal.x == 0 and avgGoal.y == 0 and avgGoal.z == 0):
                                self.addBlacklist(avgGoal)
                                self.blgoals = []

                    self.updateStatus('Stuck')
                    rospy.loginfo(self.id + ' has not moved!')

            elif not self.agent.goal.path.poses:
                # Report no path available
                self.updateStatus('No Path')
                # rospy.loginfo(self.id + ' no path!')

        checkBeacon = True
        # Manage the newest task sent
        if self.agent.guiAccept:
            if self.agent.guiTaskName == 'task':
                if self.agent.guiTaskValue == 'Explore' or self.agent.guiTaskValue == 'Start':
                    self.mode = 'Explore'
                elif self.agent.guiTaskValue == 'Home':
                    self.mode = 'Home'
                elif self.agent.guiTaskValue == 'Stop':
                    self.mode = 'Stop'
                elif self.agent.guiTaskValue == 'Goal':
                    self.mode = 'Goal'
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
            # This is only accurate if times are relatively in sync (within 60-ish seconds)
            if neighbor.lastMessage > neighbor_check:
                num_neighbors += 1

        # Publish the number of neighbors that frontier exploration should consider
        self.num_pub.publish(num_neighbors)

        # Make sure our internal artifact list is up to date, and if we need to report
        self.artifactCheck(self.agent)
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
                rospy.loginfo(self.id + ' resuming operation...')
            else:
                if self.agent.status != 'Report':
                    rospy.loginfo(self.id + ' return to report...')
                self.setGoalPoint('Report')
                # Don't check for other mode when in report
                return True

        # Check for a mode from GUI or MA, or explore normally
        if self.mode == 'Home':
            self.setGoalPoint('Home')
        elif self.mode == 'Stop':
            self.stop()
        elif self.mode == 'Deploy':
            rospy.loginfo(self.id + ' reverse deploy mode')
            if self.base.incomm:
                # Wait for a solid connection before dropping
                if self.regainBase > 5:
                    # Make sure there's no beacons already in the area
                    pose = self.agent.odometry.pose.pose
                    checkDist = self.junctionDist
                    dropBeacon = True
                    dropBeacon, nB, nDB = self.beaconDistCheck(pose, checkDist, dropBeacon)
                    if dropBeacon:
                        self.deployBeacon(True, 'Regain comms')
                    else:
                        # The bl_beacons should prevent this from occuring too many times
                        rospy.loginfo(self.id + ' beacon too close, cancelling drop')
                    self.mode = 'Explore'
                    self.regainBase = 0
                else:
                    self.regainBase += 1
            else:
                self.regainBase = 0
                self.updateStatus('Regain comms deploy')
                self.setGoalPoint('Home')
        elif self.mode == 'Goal':
            if (getDist(self.agent.odometry.pose.pose.position,
                        self.agent.guiGoalPoint.pose.position) < 1.0):
                rospy.loginfo(self.id + ' resuming exploration...')
                self.mode = 'Explore'
                # May want to add other options for tasks when it reaches the goal
                self.deconflictExplore()
            else:
                if self.agent.status != 'guiCommand':
                    rospy.loginfo(self.id + ' setting GUI Goal Point...')
                self.setGoalPoint('guiCommand')
        else:
            # Normal exploration with coordination
            self.deconflictExplore()

        # Have the aerial robots prevent maps from being merged until they're airborne
        # Useful particularly for marsupials
        # May consider using seqs to clear instead, as 'ignore' will cause the entire map
        # to get transmitted once the robot launches, but seqs means it was transmitted and
        # then thrown away...not sure best approach
        if self.isAerial and not self.launch_status and not self.agent.reset.ignore:
            rospy.loginfo('ignore aerial maps until launched')
            self.agent.reset.stamp = rospy.get_rostime()
            self.agent.reset.agent = self.id
            self.agent.reset.ignore = True
            self.agent.reset.robots = True
            self.agent.guiStamp = self.agent.reset.stamp
        elif self.isAerial and self.launch_status and self.agent.reset.ignore:
            rospy.loginfo('aerial robot launched, stop ignoring maps')
            self.agent.reset.agent = self.id
            self.agent.reset.ignore = False
            self.agent.reset.robots = True
            self.agent.reset.stamp = rospy.get_rostime()
            self.agent.guiStamp = self.agent.reset.stamp

        return True
