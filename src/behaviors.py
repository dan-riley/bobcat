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


class BCBehaviors():
    """ Establish available BOBCAT Behaviors """

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

            rospy.loginfo(self.id + ' deploying beacon ' + deploy + ' for ' + dropReason)
            try:
                # Deploy a virtual beacon whether using virtual, sim or live
                self.deploy_breadcrumb_pub.publish(Empty())

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
                    if not self.useVirtual:
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
            rospy.loginfo(self.id + ' no beacon to deploy')
            # Most likely reason it thought we had a beacon is due to restart.  So set num=0.
            self.numBeacons = 0
            self.mode = 'Explore'

    def deconflictExplore(self):
        # Explore with goal deconfliction
        self.stopStart = True
        self.agent.status = 'Explore'
        self.home_pub.publish(False)
        self.task_pub.publish(self.agent.status)
        # Find the best goal point to go to
        self.deconflictGoals()

        # If the planner can't plan, and we've reached the previous goal, or are stuck,
        # switch to trajectory follower to go towards home
        if self.useTraj or (not self.planner_status and (self.stuck > self.stopCheck or
                getDist(self.agent.goal.pose.pose.position,
                        self.agent.odometry.pose.pose.position) < 1.0)):
            self.traj_pub.publish(True)
            self.updateStatus('Following Trajectory')
            rospy.loginfo(self.id + ' using trajectory follower during explore')
            # Stop using the old goal and path or else we'll get stuck in a loop
            self.agent.goal.pose = self.agent.exploreGoal
            self.agent.goal.path = self.agent.explorePath

            # If we're stuck, with no plan, but we've reached the end of the path, blacklist this
            if len(self.agent.goal.path.poses) > 0:
                pathend = self.agent.goal.path.poses[-1].pose.position
                if (not self.planner_status and getDist(pathend,
                        self.agent.odometry.pose.pose.position) < 1.0):
                   self.addBlacklist(pathend)
        else:
            self.traj_pub.publish(False)

        # Publish the selected goal and path for the guidance controller
        self.goal_pub.publish(self.agent.goal.pose)
        self.path_pub.publish(self.agent.goal.path)
