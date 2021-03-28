#!/usr/bin/env python
from __future__ import print_function
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from util.helpers import getDist


class BCActions():
    """ Establish available BOBCAT Actions """

    def __init__(self):
        self.stopStart = True
        self.useTraj = False

        # Topics for publishers
        homeTopic = rospy.get_param('bobcat/homeTopic', 'report_artifact')
        stopTopic = rospy.get_param('bobcat/stopTopic', 'estop')
        baseCommTopic = rospy.get_param('bobcat/baseCommTopic', 'base_comm')
        goalTopic = rospy.get_param('bobcat/goalTopic', 'ma_goal')
        pathTopic = rospy.get_param('bobcat/pathTopic', 'ma_goal_path')
        self.stopCommand = rospy.get_param('bobcat/stopCommand', True)

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
        if (getDist(curpos, self.agent.exploreGoal.pose.position) > self.mappingRange and
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

    def move(self):
        # Start movement control, usually after a stop
        self.agent.status = 'Moving'
        self.task_pub.publish(self.agent.status)
        self.stop_pub.publish(not self.stopCommand)

    def stop(self):
        # Stop the robot by publishing no path, but don't change the displayed goal
        self.agent.status = 'Stop'
        self.task_pub.publish(self.agent.status)
        self.stop_pub.publish(self.stopCommand)

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
