#!/usr/bin/env python
from __future__ import print_function
import math
import rospy

from std_msgs.msg import Empty

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


class DefaultBehavior():
    """ Default Behavior object to extend for others """

    def __init__(self, agent):
        self.a = agent
        self.score = 0
        self.name = 'Default'

    def evaluate(self):
        return True

    def execute(self):
        self.score += 1

        return True


class DeployBeacon(DefaultBehavior):

    def __init__(self, agent):
        DefaultBehavior.__init__(self, agent)
        self.name = 'Deploy Beacon'

    def evaluate(self):
        self.score = 0
        # Inhibit the behavior if the Monitors say we should be reverse deploying
        if not self.a.reverseDrop:
            self.score = self.a.objectives['extendComms'].weight
        if self.a.guiBehavior == 'deployBeacon':
            self.score += self.a.objectives['input'].weight

    def execute(self):
        deploy = False
        # Find the first available beacon for this agent
        for beacon in self.a.beacons.values():
            if beacon.owner and not beacon.active:
                deploy = beacon.id
                break

        if deploy:
            # Stop the robot and publish message to deployment mechanism
            self.a.stop()
            pose = self.a.agent.odometry.pose.pose

            # TODO doesn't currently work since the node is paused!
            self.a.agent.status = 'Deploy'
            self.a.task_pub.publish(self.a.agent.status)

            if self.a.useSimComms:
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

            rospy.loginfo(self.a.id + ' deploying beacon ' + deploy + ' for ' + self.a.dropReason)
            try:
                # Deploy a virtual beacon whether using virtual, sim or live
                self.a.deploy_breadcrumb_pub.publish(Empty())

                if self.a.useSimComms:
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
                    self.a.deploy_pub.publish(True)
                    if not self.a.useVirtual:
                        rospy.sleep(10)

                # Resume the mission
                if self.a.guiBehavior == 'deployBeacon':
                    self.a.guiBehavior = None
                self.a.move()
                self.a.behaviors['explore'].execute()
                self.a.deploy_pub.publish(False)

                self.a.numBeacons = self.a.numBeacons - 1
                self.a.beacons[deploy].active = True
                self.a.beacons[deploy].simcomm = True
                self.a.beacons[deploy].pos = pose.position
            except Exception as e:
                rospy.logerr('Error deploying beacon %s', str(e))
        else:
            rospy.loginfo(self.a.id + ' no beacon to deploy')
            # Most likely reason it thought we had a beacon is due to restart.  So set num=0.
            self.a.numBeacons = 0
            if self.a.guiBehavior == 'deployBeacon':
                self.a.guiBehavior = None


class Explore(DefaultBehavior):

    def __init__(self, agent):
        DefaultBehavior.__init__(self, agent)
        self.name = 'Explore'

    def evaluate(self):
        self.score = self.a.objectives['explore'].weight

    def execute(self):
        # Explore with goal deconfliction
        self.a.stopStart = True
        # Reduce all the redundant explore messages
        if self.a.agent.status != 'Explore':
            self.a.agent.status = 'Explore'
            self.a.home_pub.publish(False)
            self.a.task_pub.publish(self.a.agent.status)

        # Find the best goal point to go to, unless we need a blacklist replan
        if not self.a.blacklistUpdated:
            self.a.deconflictGoals()

        # If a replan was requested somewhere, trigger it
        if self.a.replan or self.a.blacklistUpdated:
            self.a.updateStatus('Replanning')
            rospy.loginfo(self.a.id + ' requesting replan')
            if self.a.blacklistUpdated:
                self.a.task_pub.publish('unstuck')
            else:
                self.a.task_pub.publish(self.a.replanCommand)
                self.a.replan = False

        # If the planner can't plan, and we've reached the previous goal, or are stuck,
        # switch to trajectory follower to go towards home
        if self.a.useTraj or (not self.a.planner_status and (self.a.stuck > self.a.stopCheck or
                getDist(self.a.agent.goal.pose.pose.position,
                        self.a.agent.odometry.pose.pose.position) < 1.0)):
            self.a.traj_pub.publish(True)
            # Try to get the planner to replan
            self.a.task_pub.publish(self.a.replanCommand)
            self.a.updateStatus('Following Trajectory')
            rospy.loginfo(self.a.id + ' using trajectory follower during explore')
            # Stop using the old goal and path or else we'll get stuck in a loop
            self.a.agent.goal.pose = self.a.agent.exploreGoal
            self.a.agent.goal.path = self.a.agent.explorePath

            # If we're stuck, with no plan, but we've reached the end of the path, blacklist this
            if len(self.a.agent.goal.path.poses) > 0:
                pathend = self.a.agent.goal.path.poses[-1].pose.position
                if (not self.a.planner_status and getDist(pathend,
                        self.a.agent.odometry.pose.pose.position) < 1.0):
                   self.a.addBlacklist(pathend)
        else:
            self.a.traj_pub.publish(False)

        # Publish the selected goal and path for the guidance controller
        self.a.goal_pub.publish(self.a.agent.goal.pose)
        if self.a.agent.goal.path.header.frame_id != 'starting':
            self.a.path_pub.publish(self.a.agent.goal.path)


class GoToGoal(DefaultBehavior):

    def __init__(self, agent):
        DefaultBehavior.__init__(self, agent)
        self.name = 'Go to Goal'

    def evaluate(self):
        self.score = 0
        if self.a.guiBehavior == 'goToGoal':
            self.score = self.a.objectives['input'].weight

    def execute(self):
        # This part should probably be in a Monitor instead
        if (getDist(self.a.agent.odometry.pose.pose.position,
                    self.a.agent.guiGoalPoint.pose.position) < 1.0):
            rospy.loginfo(self.a.id + ' resuming exploration...')
            self.a.guiBehavior = None
            # May want to add other options for tasks when it reaches the goal
            self.a.behaviors['explore'].execute()
        else:
            if self.a.agent.status != 'guiCommand':
                rospy.loginfo(self.a.id + ' setting GUI Goal Point...')
            self.a.setGoalPoint('guiCommand')


class GoHome(DefaultBehavior):

    def __init__(self, agent):
        DefaultBehavior.__init__(self, agent)
        self.name = 'Go Home'

    def evaluate(self):
        # Reporting and Maintaining comms primarily drive this behavior
        self.score = self.a.objectives['maintainComms'].weight + self.a.objectives['report'].weight

        # Extend comms also drives if the monitor says to reverse drop
        if self.a.reverseDrop:
            self.score += self.a.objectives['extendComms'].weight
        if self.a.guiBehavior == 'home':
            self.score += self.a.objectives['input'].weight

    def execute(self):
        reason = 'Home'
        if self.a.report:
            rospy.loginfo(self.a.id + ' return to report...')
            reason = 'Report'
        if self.a.reverseDrop:
            rospy.loginfo(self.a.id + ' reverse deploy mode')
            self.a.updateStatus('Regain comms deploy')
        self.a.setGoalPoint(reason)


class Stop(DefaultBehavior):

    def __init__(self, agent):
        DefaultBehavior.__init__(self, agent)
        self.name = 'Stop'

    def evaluate(self):
        self.score = 0
        if self.a.guiBehavior == 'stop':
            self.score = self.a.objectives['input'].weight

    def execute(self):
        self.a.stop()
