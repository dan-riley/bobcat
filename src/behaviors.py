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
        self.name = self.__class__.__name__
        self.monitors = []
        self.objectives = []

    def evaluate(self):
        return True

    def execute(self):
        self.score += 1

        return True


class DeployBeacon(DefaultBehavior):

    def __init__(self, agent):
        DefaultBehavior.__init__(self, agent)
        self.monitors = ['ReverseDrop', 'HumanInput']
        self.objectives = ['ExtendComms', 'Input']

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
            # Publish message to guidance/deployment mechanism
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
                # New guidance publishes this now after maneuver, but leaving if needed later
                # self.a.deploy_breadcrumb_pub.publish(Empty())

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
                    # Send deploy message; guidance stops and maneuvers, we just continue
                    self.a.deploy_pub.publish(True)
                    rospy.sleep(1)

                # Resume the mission
                if self.a.guiBehavior == 'deployBeacon':
                    self.a.guiBehavior = None
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
        self.monitors = ['HumanInput']  # Implict based on implementation
        self.objectives = ['Explore']

    def evaluate(self):
        self.score = self.a.objectives['explore'].weight

    def execute(self):
        self.a.explore()


class GoToGoal(DefaultBehavior):

    def __init__(self, agent):
        DefaultBehavior.__init__(self, agent)
        self.monitors = ['HumanInput']
        self.objectives = ['Input']

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
        self.monitors = ['ReverseDrop', 'HumanInput']
        self.objectives = ['MaintainComms', 'ExtendComms', 'ReportArtifacts', 'Input']

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
        self.monitors = ['HumanInput']
        self.objectives = ['Input']

    def evaluate(self):
        self.score = 0
        if self.a.guiBehavior == 'stop':
            self.score = self.a.objectives['input'].weight

    def execute(self):
        self.a.stop()
