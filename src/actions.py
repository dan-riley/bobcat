#!/usr/bin/env python
from __future__ import print_function
import math
import copy
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from bobcat.msg import BobcatStatus, Monitor, Objective, Behavior

from util.helpers import getDist, getYaw

# Import Ignition/Gazebo only if running in the sim so the robot doesn't need them
if rospy.get_param('bobcat/simcomms', False):
    # Switch for ignition or gazebo here temporarily until I find a better place
    useIgnition = True

    from gazebo_msgs.msg import ModelState
    if useIgnition:
        from subt_msgs.srv import SetPose
    else:
        from gazebo_msgs.srv import SetModelState


class BCActions():
    """ Establish available BOBCAT Actions """

    def __init__(self):
        self.stopStart = True
        self.useTraj = False
        self.trajOn = False
        self.lastBeacon = False
        self.lastGoalTime = rospy.Time()
        self.lastReplanTime = rospy.Time(0)
        self.lastGoalTime = rospy.Time()

        # Topics for publishers
        homeTopic = rospy.get_param('bobcat/homeTopic', 'report_artifact')
        stopTopic = rospy.get_param('bobcat/stopTopic', 'estop')
        baseCommTopic = rospy.get_param('bobcat/baseCommTopic', 'base_comm')
        goalTopic = rospy.get_param('bobcat/goalTopic', 'ma_goal')
        pathTopic = rospy.get_param('bobcat/pathTopic', 'ma_goal_path')
        self.stopCommand = rospy.get_param('bobcat/stopCommand', True)
        self.replanCommand = rospy.get_param('bobcat/replanCommand', 'deconflict')
        self.singleGoalDeconflict = rospy.get_param('bobcat/singleGoalDeconflict', False)
        self.useExtTraj = rospy.get_param('bobcat/useExtTraj', False)

        self.status_pub = rospy.Publisher('bobcat_status', BobcatStatus, latch=True, queue_size=1)
        self.task_pub = rospy.Publisher('task', String, queue_size=10, latch=True)
        self.deploy_pub = rospy.Publisher('deploy_beacon', Bool, queue_size=10)
        self.deploy_breadcrumb_pub = rospy.Publisher('breadcrumb/deploy', Empty, queue_size=10)
        self.reset_pub = rospy.Publisher('reset_artifacts', Bool, queue_size=10)
        self.num_pub = rospy.Publisher('num_neighbors', Int8, queue_size=10)
        self.traj_pub = rospy.Publisher('follow_traj', Bool, queue_size=10)
        self.home_pub = rospy.Publisher(homeTopic, Bool, queue_size=10)
        self.stop_pub = rospy.Publisher(stopTopic, Bool, queue_size=10)
        self.comm_pub = rospy.Publisher(baseCommTopic, Bool, queue_size=10)
        self.goal_pub = rospy.Publisher(goalTopic, PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher(pathTopic, Path, queue_size=10)

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

    def publishGUITask(self):
        if self.agent.guiTaskValue == 'True':
            data = True
        elif self.agent.guiTaskValue == 'False':
            data = False
        else:
            data = self.agent.guiTaskValue

        self.pub_guiTask[self.agent.guiTaskName].publish(data)

    def updateStatus(self, status):
        if self.newStatus and status not in self.newStatus:
            self.newStatus += '+++' + status
        else:
            self.newStatus = status

    def checkStatus(self, status):
        if self.newStatus and status in self.newStatus:
            return True

        return False

    def checkBlacklist(self, goal):
        addBlacklist = True
        # Make sure it's not already in a blacklist radius
        for bgoal in self.blacklist.points:
            if getDist(bgoal, goal) < self.deconflictRadius:
                addBlacklist = False

        return addBlacklist

    def addBlacklist(self, goal):
        if self.checkBlacklist(goal):
            # If this is our first new blacklist, set the reset timer
            if not self.blacklist.points:
                self.blacklistResetTime = rospy.get_rostime() + rospy.Duration(20)
            goalstr = str(goal.x) + '-' + str(goal.y) + '-' + str(goal.z)
            rospy.loginfo(self.id + ' added ' + goalstr + ' to blacklist')
            self.blacklist.points.append(goal)
            self.pub_blacklist.publish(self.blacklist)

    def dropBeacon(self):
        # A beacon was deployed
        if self.lastBeacon and self.beaconDeployed:
            rospy.loginfo(self.id + ' deployed beacon ' + self.lastBeacon)
            self.numBeacons = self.numBeacons - 1

            # Calculate the beacon position
            pose = copy.deepcopy(self.agent.odometry.pose.pose)
            yaw = getYaw(pose.orientation)
            center = (self.maxBeacons - 1.0) / 2.0
            # Get beacon position number so we can go left to right
            bpos = self.maxBeacons - self.startBeacons + self.numBeacons
            ratio = (bpos - center) / center
            # Angle in radians for max +/- 30 degrees
            angrad = ratio * 0.5236
            # Adjust for the yaw of the robot
            yaw -= angrad
            # Adjust the length for the angle
            offset = 0.5 / math.cos(angrad)
            # Set the position
            pose.position.x -= math.cos(yaw) * offset
            pose.position.y -= math.sin(yaw) * offset
            pose.position.z -= 0.1

            # Store the beacon
            self.beacons[self.lastBeacon].active = True
            self.beacons[self.lastBeacon].simcomm = True
            self.beacons[self.lastBeacon].pos = pose.position

            # Reset monitors
            self.deployBeacon = False
            self.beaconDeployed = False
            self.lastBeacon = False

            # Resume the mission
            if self.guiBehavior == 'deployBeacon':
                self.guiBehavior = None
            return
        elif self.lastBeacon:
            # A beacon has been requested but not deployed
            self.updateGoalPath(True)
            return

        deploy = False
        # Find the first available beacon for this agent
        for beacon in self.beacons.values():
            if beacon.owner and not beacon.active:
                deploy = beacon.id
                break

        if deploy:
            # Publish message to guidance/deployment mechanism
            pose = copy.deepcopy(self.agent.odometry.pose.pose)

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

                offset = 0.5

                pose.position.x = pose.position.x - math.cos(yaw) * offset
                pose.position.y = pose.position.y - math.sin(yaw) * offset

                state = ModelState()
                state.model_name = deploy
                state.pose = pose

            rospy.loginfo(self.id + ' deploying beacon ' + deploy + ' for ' + self.dropReason)
            try:
                # Deploy a virtual beacon whether using virtual, sim or live
                # New guidance publishes this now after maneuver, but leaving if needed later
                # self.deploy_breadcrumb_pub.publish(Empty())

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

                # Send deploy message; guidance stops and maneuvers, we just continue
                self.move()
                self.deploy_pub.publish(True)
                self.lastBeacon = deploy

                # Ask the the planner to go ahead and start planning a path
                if self.exploreToGoal:
                    self.updatePlannerGoal()
                else:
                    self.task_pub.publish('Explore')
            except Exception as e:
                rospy.logerr('Error deploying beacon %s', str(e))
        else:
            rospy.loginfo(self.id + ' no beacon to deploy')
            # Most likely reason it thought we had a beacon is due to restart.  So set num=0.
            self.numBeacons = 0
            if self.guiBehavior == 'deployBeacon':
                self.guiBehavior = None

    def publishGoalPath(self):
        self.goal_pub.publish(self.agent.goal.pose)
        if self.agent.goal.path.header.frame_id != 'starting':
            self.path_pub.publish(self.agent.goal.path)

    def updateGoalPath(self, publish=False):
        self.agent.goal.pose = self.agent.exploreGoal
        self.agent.goal.path = self.agent.explorePath
        if publish:
            self.publishGoalPath()

    ##### Begin Explore actions #####
    def deconflictGoals(self):
        # Get all of the goals into a list
        goals = self.agent.goals.goals
        # Last goal and path chosen
        curgoal = self.agent.goal.pose.pose.position
        curpos = self.agent.odometry.pose.pose.position
        expgoal = self.agent.exploreGoal.pose.position
        # Keep going to long distance goal, unless it's home, or we get a new potential goal close
        if (getDist(curpos, expgoal) > self.mappingRange and
                getDist(curpos, curgoal) > self.mappingRange + 2.0 and
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
                        self.updateGoalPath()
                else:
                    for goal in goals:
                        if getDist(curgoal, goal.pose.pose.position) < 0.5:
                            self.agent.goal = goal
                            break

                rospy.loginfo(self.id + ' continuing to long distance goal')
                return

        if (self.singleGoalDeconflict and
                not (expgoal.x == 0 and expgoal.y == 0 and expgoal.z == 0)):
            # Only getting one goal point so have to request a replan if there's a conflict
            gpos = self.agent.exploreGoal.pose.position
            godom = self.agent.odometry.pose.pose.position

            # Global planner should take care of this now, but it's a double check
            for goal in self.blacklist.points:
                if getDist(gpos, goal) < self.deconflictRadius:
                    self.blacklistUpdated = 'blacklist'
                    break

            if not self.blacklistUpdated:
                # Check each neighbors' goal for conflict
                for neighbor in self.neighbors.values():
                    # Ignore the neighbor if we don't have current data
                    # Requires clocks to be relatively in-sync
                    if rospy.get_rostime() > neighbor.lastMessage + rospy.Duration(15):
                        continue
                    npos = neighbor.goal.pose.pose.position
                    nodom = neighbor.odometry.pose.pose.position
                    # Check whether they are within the defined range
                    if getDist(gpos, npos) < self.deconflictRadius:
                        # If our cost is more than the neighbor, don't go to this goal
                        if getDist(gpos, godom) > getDist(npos, nodom):
                            self.replan = 'neighbor'
                            # Don't need to check any more neighbors for this goal if conflict
                            break

            # Set the goal to the planner's goal
            self.updateGoalPath()
        elif not goals:
            # Set the goal to the frontier goal
            # This should cause us to navigate to the goal, then go home if still no plan
            self.updateGoalPath()
        elif len(goals) == 1:
            # If we only have one potential goal, just go there
            # May want to consider stopping in place if there is a conflict!
            self.agent.goal = goals[0]
        else:
            # Receiving multiple goals from planner, so we can choose one directly
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
                        # Ignore the neighbor if we don't have current data
                        # Requires clocks to be relatively in-sync
                        if rospy.get_rostime() > neighbor.lastMessage + rospy.Duration(15):
                            continue
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
                    self.updateGoalPath()
                    self.blacklist.points = []
                    rospy.loginfo(self.id + ' all goals blacklisted ' + str(len(goals)))
                    self.useTraj = True
            else:
                # Set the goal to the last goal without conflict
                self.agent.goal = goals[i - 1]

        # Final blacklist check for single goal
        if not self.singleGoalDeconflict and (not goals or len(goals) == 1):
            goal = self.agent.goal.pose.pose.position
            if len(self.agent.goal.path.poses) > 0:
                pathend = self.agent.goal.path.poses[-1].pose.position
            else:
                pathend = goal

            self.useTraj = False
            if not self.checkBlacklist(goal) or not self.checkBlacklist(pathend):
                rospy.loginfo(self.id + ' only goal is blacklisted, using trajectory follower')
                self.useTraj = True

    def replanStatus(self, reason):
        if reason == 'neighbor':
            self.updateStatus('Replanning Neighbor')
            rospy.loginfo(self.id + ' replanning due to neighbor')
        elif reason == 'neighborPath':
            self.updateStatus('Replanning Neighbor Path')
            rospy.loginfo(self.id + ' tired of waiting, replanning due to neighbor')
        elif reason == 'gui':
            self.updateStatus('Replanning GUI')
            rospy.loginfo(self.id + ' replanning due to GUI input')
        elif reason == 'blacklist':
            self.updateStatus('Replanning Blacklist')
            rospy.loginfo(self.id + ' replanning due to existing blacklist')
        elif reason == 'newBlacklist':
            self.updateStatus('Replanning New Blacklist')
            rospy.loginfo(self.id + ' replanning due to new blacklist')
        else:
            self.updateStatus('Replanning')
            rospy.loginfo(self.id + ' replanning')

    def replanCheck(self):
        # If a replan was requested somewhere, trigger it, unless we already asked recently
        if (rospy.get_rostime() > self.lastReplanTime + rospy.Duration(self.stopCheck) and
                self.startedMission and (self.replan or self.blacklistUpdated)):
            self.lastReplanTime = rospy.get_rostime()
            if self.blacklistUpdated:
                self.replanStatus(self.blacklistUpdated)
                self.task_pub.publish('unstuck')
            else:
                # Ignore stale neighbor path replans
                if not self.neighborWait and self.replan == 'neighborPath':
                    return False
                # When stopped for a neighbor path, briefly get planner in explore mode
                if self.replan == 'neighborPath':
                    self.task_pub.publish('Explore')
                    rospy.sleep(1)
                self.replanStatus(self.replan)
                self.task_pub.publish(self.replanCommand)
                self.replan = False

            return True

        return False

    def trajCheck(self, alreadyReplanned):
        # If the planner can't plan, and we've reached the previous goal, or are stuck,
        # switch to trajectory follower to go towards home
        if self.useTraj or (not self.planner_status and (self.stuck > self.stopCheck or
                getDist(self.agent.goal.pose.pose.position,
                        self.agent.odometry.pose.pose.position) < 1.0)):
            # Try to get the planner to replan
            if (not alreadyReplanned and
                    rospy.get_rostime() > self.lastReplanTime + rospy.Duration(self.stopCheck)):
                self.task_pub.publish(self.replanCommand)
            if self.useExtTraj and not self.trajOn:
                self.updateStatus('Following Trajectory')
                rospy.loginfo(self.id + ' using trajectory follower during explore')
                self.trajOn = True
                self.traj_pub.publish(True)
            # Stop using the old goal and path or else we'll get stuck in a loop
            self.updateGoalPath()

            # If we're stuck, with no plan, but we've reached the end of the path, blacklist this
            if len(self.agent.goal.path.poses) > 0:
                pathend = self.agent.goal.path.poses[-1].pose.position
                if (self.startedMission and not self.planner_status and
                        getDist(pathend, self.agent.odometry.pose.pose.position) < 1.0):
                   self.addBlacklist(pathend)
        elif self.trajOn:
            self.trajOn = False
            self.traj_pub.publish(False)

    def explore(self):
        # Explore with goal deconfliction
        self.stopStart = True
        # Reduce all the redundant explore messages
        if self.agent.status != 'Explore':
            self.move()
            self.agent.status = 'Explore'
            self.home_pub.publish(False)
            self.task_pub.publish(self.agent.status)

        # Find the best goal point to go to, unless we need a blacklist replan
        if not self.blacklistUpdated:
            self.deconflictGoals()

        # Check for replanning and whether we need to trajectory follow
        self.trajCheck(self.replanCheck())

        # Publish the selected goal and path
        self.publishGoalPath()
    ##### End Explore actions #####

    def move(self, pauseCheck=False):
        if pauseCheck and not self.paused:
            return
        # Start movement control, usually after a stop
        self.paused = False
        self.ignoreStopCommand = True
        self.stop_pub.publish(not self.stopCommand)

    def pause(self):
        # Stop the robot momentarily but don't change any statuses
        self.paused = True
        self.ignoreStopCommand = True
        self.stop_pub.publish(self.stopCommand)
        self.stopUpdate()

    def stop(self):
        # Stop the robot by publishing no path, but don't change the displayed goal
        if self.agent.status != 'Stop':
            self.ignoreStopCommand = True
            self.agent.status = 'Stop'
            self.task_pub.publish(self.agent.status)
            self.stop_pub.publish(self.stopCommand)

        if self.nearbyRobot:
            self.updateStatus('Waiting')
            rospy.loginfo(self.id + ' waiting for neighbor to move...')
            self.neighborWait += 1
            if self.neighborWait > self.stopCheck:
                self.replan = 'neighborPath'
                self.replanCheck()

        self.stopUpdate()

    def stopUpdate(self):
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
        else:
            self.updateGoalPath(True)

    def updatePlannerGoal(self):
        # Have the planner plan to the goal point while we do other things
        self.pub_guiGoal.publish(self.agent.guiGoalPoint)
        self.task_pub.publish('guiCommand')

    def setGoalPoint(self, reason):
        # If we're stuck, briefly go to explore
        if self.blacklistUpdated:
            self.blacklistUpdated = False
            self.explore()
            return

        # Set the goal point for frontier exploration
        if reason == 'guiCommand':
            # Publishing resets the seq, but we're using that to track new commands, so save it
            seq = self.agent.guiGoalPoint.header.seq
            self.pub_guiGoal.publish(self.agent.guiGoalPoint)
            self.agent.guiGoalPoint.header.seq = seq
            # Prevent getting stuck going to an unreachable goal
            if rospy.get_rostime() > self.lastGoalTime + rospy.Duration(180):
                rospy.loginfo('GUI Goal timeout, going back to explore')
                self.guiBehavior = None
                self.exploreToGoal = False
        else:
            # For now Home and Report need this set to switch frontier exploration
            self.home_pub.publish(True)

        # Set the new task, and use frontier exploration's goal and path
        # Republish the command periodically in case planner went somewhere else
        self.stopStart = True
        if (not self.paused and (self.agent.status != reason or
                rospy.get_rostime() > self.lastGoalTime + rospy.Duration(self.stopCheck))):
            self.lastGoalTime = rospy.get_rostime()
            self.move()
            self.agent.status = reason
            self.task_pub.publish(self.agent.status)
        self.updateGoalPath(True)

        # Use the external trajectory follower to go home if planner refuses
        # Don't ask immediately for a replan, the command will be sent periodically
        # This allows us to move a bit before requesting the plan again
        if self.useExtTraj and not self.planner_status and reason != 'guiCommand':
            if not self.trajOn:
                self.trajOn = True
                self.traj_pub.publish(True)
                self.updateStatus('Following Trajectory')
                rospy.loginfo(self.id + ' using trajectory follower for home')
        elif self.trajOn:
            self.trajOn = False
            self.traj_pub.publish(False)

    def buildMonitorStatus(self, status, name, value):
        mon = Monitor()
        mon.name = name
        mon.status = value
        status.monitors.append(mon)

    def publishStatus(self, execBehavior):
        status = BobcatStatus()
        status.header.stamp = rospy.get_rostime()

        # Build each monitor individually for now since they're not as easily looped
        status.inputCommand = str(self.guiBehavior)
        self.buildMonitorStatus(status, 'Artifact', self.report)
        self.buildMonitorStatus(status, 'HumanInput', self.guiBehavior != None)
        self.buildMonitorStatus(status, 'Comms', self.base.incomm)
        self.buildMonitorStatus(status, 'ExploreToGoal', self.exploreToGoal)
        self.buildMonitorStatus(status, 'Beacon', self.deployBeacon)
        self.buildMonitorStatus(status, 'ReverseDrop', self.reverseDrop)
        self.buildMonitorStatus(status, 'NearbyRobot', self.nearbyRobot)

        # Get the objectives and behaviors
        for objective in self.objectives.values():
            obj = Objective()
            obj.name = objective.name
            obj.monitors = objective.monitors
            obj.weight = objective.weight
            status.objectives.append(obj)

        for behavior in self.behaviors.values():
            beh = Behavior()
            beh.name = behavior.name
            # Including the connections every message is overkill but also adds flexibility
            beh.monitors = behavior.monitors
            beh.objectives = behavior.objectives
            beh.score = behavior.score
            status.behaviors.append(beh)

        status.execBehavior = execBehavior
        self.status_pub.publish(status)
