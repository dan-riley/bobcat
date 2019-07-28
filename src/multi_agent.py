#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import copy
import pickle
import zlib
import rospy
import tf

from std_msgs.msg import Bool
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from marble_artifact_detection_msgs.msg import ArtifactArray
from marble_multi_agent.msg import CommsCheckArray
from marble_multi_agent.msg import Beacon
from marble_multi_agent.msg import BeaconArray


class Neighbor(object):
    """ Data structure to hold pertinent information about other agents """

    def __init__(self, agent_id):
        self.id = agent_id
        self.cid = String()
        self.odometry = Odometry()
        self.goal = PoseStamped()
        self.map = MarkerArray()
        self.commBeacons = BeaconArray()
        self.newArtifacts = ArtifactArray()
        self.lastMessage = rospy.get_rostime()
        self.lastCommCheck = rospy.get_rostime()
        self.incomm = True
        self.simcomm = True

        """ Other properties that need to eventually be added for full functionality:
        self.cid = id of the agent actually in direct comm with this neighbor, if it's indirect
        self.type = robot, beacon, etc.
        self.goalType = frontier, anchor, follow, wait, etc.
        self.cost = cost to reach the goal
        self.stateHistory = previous path of the agent for display purposes mostly
        self.path = the intended path of the agent for display purposes mostly
        self.replan = boolean flag for whether to force this agent to replan on it's next cycle
        self.incomm = boolean flag whether the agent is currently in comm, useful only at anchor
        """

    def update(self, neighbor, updater=False):
        self.odometry = neighbor.odometry
        self.goal = neighbor.goal

        # Update parameters depending on if we're talking directly or not
        if updater:
            self.cid = updater
            self.lastMessage = rospy.get_rostime()
            self.incomm = True
        else:
            self.cid = neighbor.cid
            self.lastMessage = neighbor.lastMessage
            self.incomm = False

    # def update(self, pos, goal, goalType, cost):
    #     self.pos = pos
    #     self.goal = goal
    #     self.goalType = goalType
    #     self.cost = cost
    #     self.incomm = True
    #
    # def history(self, stateHistory, path):
    #     self.stateHistory = stateHistory
    #     self.path = path


class Agent(Neighbor):
    """ Sub-class for an individual agent, with an array of neighbors to pass data """

    def __init__(self, agent_id):
        super(Agent, self).__init__(agent_id)
        self.neighbors = {}


class BeaconObj(object):
    """ Data structure to hold pertinent information about beacons """

    def __init__(self, agent):
        self.id = agent
        self.pos = Point()
        self.incomm = True
        self.simcomm = True
        self.active = False
        self.lastCommCheck = rospy.get_rostime()


class ArtifactReport:
    """
    Internal artifact structure to track reporting.
    Holds full artifact message so neighbors can be fused.
    """

    def __init__(self, artifact, artifact_id):
        self.id = artifact_id
        self.artifact = artifact
        self.reported = False


class DataListener:
    """ Listens to all of the applicable topics and repackages into a single object """

    def __init__(self, agent, agent_type):
        self.agent = agent  # Agent or Neighbor object
        self.odom_sub = rospy.Subscriber(self.agent.id + '/odometry', Odometry, self.Receiver, 'odometry', queue_size=1)
        self.goal_sub = rospy.Subscriber(self.agent.id + '/frontier_goal_pose', PoseStamped, self.Receiver, 'goal', queue_size=1)
        self.beacon_sub = rospy.Subscriber(self.agent.id + '/beacons', BeaconArray, self.Receiver, 'commBeacons', queue_size=1)
        self.artifact_sub = rospy.Subscriber(self.agent.id + '/artifact_array', ArtifactArray, self.Receiver, 'newArtifacts', queue_size=1)

        if agent_type == 'base':
            self.map_sub = rospy.Subscriber(self.agent.id + '/occupied_cells_vis_array', MarkerArray, self.Receiver, 'map')
            # self.map_sub = rospy.Subscriber(self.agent.id + '/voxblox_node/occupied_nodes', MarkerArray, self.MapReceiver, 'map')

    def Receiver(self, data, parameter):
        if self.agent.simcomm:
            setattr(self.agent, parameter, data)

        # Throttle our updates to at least faster than the multi-agent node
        # Don't throttle an octomap or it doesn't update properly
        if parameter != 'map':
            rospy.sleep(0.5)


class MultiAgent:
    """ Initialize a multi-agent node for the agent, publishes data for others and listens """

    def __init__(self, agent_id, agent_type):
        self.id = agent_id
        self.type = agent_type
        self.neighbors = {}
        self.beacons = {}
        self.comm_sub = {}
        self.simcomms = {}
        self.commcheck = {}
        self.artifacts = {}
        self.monitor = {}
        self.maxDist = 20  # distance to drop beacons automatically
        self.useSimComms = True
        self.report = False

        # Define which nodes republish information to display
        self.monitors = ['Anchor']

        # Setup possible beacons to deploy for this agent
        if self.type == 'robot':
            self.numBeacons = 2
        else:
            self.numBeacons = 0

        rospy.init_node(self.id + '_multi_agent')
        self.start_time = rospy.get_rostime()
        while self.start_time.secs == 0:
            self.start_time = rospy.get_rostime()

        self.agent = Agent(self.id)

        # Setup the internal listeners if we're a robot
        if self.type == 'robot':
            DataListener(self.agent, 'robot')
            self.task_pub = rospy.Publisher('/' + self.id + '/task', String, queue_size=10)
            self.stop_pub = rospy.Publisher('/' + self.id + '/stop', Bool, queue_size=10)
            self.stop = Bool()
            self.stop.data = False

        if self.useSimComms:
            comm_topic = '/' + self.id + '/commcheck'
            self.comm_sub[self.id] = rospy.Subscriber(comm_topic, CommsCheckArray, self.CommChecker, self.id)

        # Setup the beacons.  For real robots the names shouldn't matter as long as consistent
        # May need to code the possible ones for each robot though so they don't deploy
        # duplicate names...but if they only deploy when connected to mesh it shouldn't matter
        for i in range(1, 5):
            nid = 'B' + str(i)
            self.beacons[nid] = BeaconObj(nid)
            if self.useSimComms:
                comm_topic = '/' + nid + '/commcheck'
                self.comm_sub[nid] = rospy.Subscriber(comm_topic, CommsCheckArray, self.CommChecker, nid)

        # Setup all of the neighbors.  This should either be hardcoded or added as a parameter
        neighbors = ['Anchor', 'X1', 'X2']

        # Setup the listeners for each neighbor
        for nid in [n for n in neighbors if n != self.id]:
            self.neighbors[nid] = Neighbor(nid)
            DataListener(self.neighbors[nid], self.type)

            if self.useSimComms:
                comm_topic = '/' + nid + '/commcheck'
                self.comm_sub[nid] = rospy.Subscriber(comm_topic, CommsCheckArray, self.CommChecker, nid)

            # Setup topics for visualization at whichever monitors are specified (usually base)
            if self.id in self.monitors:
                topic = '/' + self.id + '/neighbors/' + nid + '/'
                self.monitor[nid] = {}
                self.monitor[nid]['odometry'] = rospy.Publisher(topic + 'odometry', Odometry, queue_size=10)
                self.monitor[nid]['goal'] = rospy.Publisher(topic + 'goal', PoseStamped, queue_size=10)
                self.monitor[nid]['map'] = rospy.Publisher(topic + 'map', MarkerArray, queue_size=10)

        self.beacon_pub = rospy.Publisher('/' + self.id + '/beacons', BeaconArray, queue_size=10)

        # Setup the beacon monitor
        if self.id in self.monitors:
            self.monitor['beacons'] = rospy.Publisher('/' + self.id + '/mbeacons', Marker, queue_size=10)
            self.mbeacon = Marker()
            self.mbeacon.header.frame_id = 'world'
            self.mbeacon.type = self.mbeacon.CUBE_LIST
            self.mbeacon.action = self.mbeacon.ADD
            self.mbeacon.scale.x = 1.0
            self.mbeacon.scale.y = 1.0
            self.mbeacon.scale.z = 1.0
            self.mbeacon.color.a = 1.0
            self.mbeacon.color.r = 1.0
            self.mbeacon.color.g = 1.0
            self.mbeacon.color.b = 1.0

    def publishNeighbors(self):
        # Topics for visualization at the anchor node
        for neighbor in self.neighbors.values():
            self.monitor[neighbor.id]['odometry'].publish(neighbor.odometry)
            self.monitor[neighbor.id]['goal'].publish(neighbor.goal)
            self.monitor[neighbor.id]['map'].publish(neighbor.map)

        self.mbeacon.points = []
        for beacon in self.beacons.values():
            self.mbeacon.points.append(beacon.pos)

        self.monitor['beacons'].publish(self.mbeacon)

    def publishBeacons(self):
        commBeacons = []
        for beacon in self.beacons.values():
            # TODO use last message timer...will be tricky!
            beacon.incomm = beacon.simcomm
            commBeacon = Beacon()
            commBeacon.id = beacon.id
            commBeacon.active = beacon.active
            commBeacon.pos = beacon.pos
            commBeacons.append(commBeacon)

        self.beacon_pub.publish(commBeacons)

    def CommChecker(self, data, nid):
        # If we're checking our direct comm, set simcomm directly
        if nid == self.id:
            for neighbor in data.data:
                self.simcomms[neighbor.id] = neighbor
        else:
            # Otherwise build our multidimensional checking array
            self.commcheck[nid] = data.data

    def recurCommCheck(self, cid):
        for check in self.commcheck[cid]:
            if check.incomm and check.id != self.id and not self.simcomms[check.id].incomm:
                self.simcomms[check.id].incomm = True
                self.recurCommCheck(check.id)

    def simCommCheck(self):
        # Recursively check who can talk to who
        for cid in self.commcheck:
            if self.simcomms[cid].incomm:
                self.recurCommCheck(cid)

        # Set the simcomm based on the newly modified matrix
        for simcomm in self.simcomms.values():
            if 'B' in simcomm.id:
                self.beacons[simcomm.id].simcomm = simcomm.incomm
            else:
                self.neighbors[simcomm.id].simcomm = simcomm.incomm

    def spawnBeacon(self, inplace):
        spawn = False
        # Find the first available beacon for this agent
        # As long as two agents don't try to drop beacons at the exact same time this is ok
        for beacon in self.beacons.values():
            if not beacon.active:
                spawn = beacon.id
                break

        if spawn:
            self.stop.data = True
            self.stop_pub.publish(self.stop)

            # For now we're just moving the beacon 3 units behind which will hopefull
            # be in range of the anchor or previous beacon
            # Either need to identify location before comm loss, or make this a guidance
            # command to return to a point in range
            service = '/gazebo/set_model_state'
            rospy.wait_for_service(service)
            set_state = rospy.ServiceProxy(service, SetModelState)

            pose = self.agent.odometry.pose.pose
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w])

            if inplace:
                offset = 1
            else:
                offset = 6

            pose.position.x = pose.position.x - math.cos(yaw) * offset
            pose.position.y = pose.position.y - math.sin(yaw) * offset

            state = ModelState()
            state.model_name = spawn
            state.pose = pose

            print("moving beacon", spawn)
            try:
                ret = set_state(state)
                self.numBeacons = self.numBeacons - 1
                self.beacons[spawn].active = True
                self.beacons[spawn].incomm = True
                self.beacons[spawn].simcomm = True
                self.beacons[spawn].pos = pose.position
                print(ret.status_message)
            except Exception, e:
                rospy.logerr('Error calling service: modestate %s', str(e))
        else:
            print("no beacon to spawn")

    def beaconCheck(self):
        # Check if we need to drop a beacon if we have any beacons to drop
        if self.numBeacons > 0:
            myPos = self.agent.odometry.pose.pose.position
            numBeacons = 0
            dropBeacon = False
            backBeacon = []
            # Find how many beacons are active and that we can talk to
            for beacon in self.beacons.values():
                if beacon.active:
                    backBeacon.append(False)
                    # Find the ones we're in comm with, and how far they are
                    if beacon.incomm:
                        numBeacons = numBeacons + 1
                        pos = beacon.pos
                        dist = math.sqrt((myPos.x - pos.x)**2 + (myPos.y - pos.y)**2 + (myPos.z - pos.z)**2)

                        if dist > self.maxDist:
                            dropBeacon = True

                    else:
                        backBeacon[-1] = True

            if len(backBeacon) == 0:
                backBeacon = [False]

            # If there are no beacons, and we're talking to anchor,
            # check to see if we're getting far away and need to drop one
            if self.neighbors['Anchor'].incomm and numBeacons == 0:
                anchorPos = self.neighbors['Anchor'].odometry.pose.pose.position
                anchorDist = math.sqrt((myPos.x - anchorPos.x)**2 + (myPos.y - anchorPos.y)**2 + (myPos.z - anchorPos.z)**2)

                if anchorDist > self.maxDist and numBeacons == 0:
                    print("anchor distance limit, let's drop!")
                    dropBeacon = True

            # If there are no beacons and we've lost comm we should try to drop
            elif not self.neighbors['Anchor'].incomm and numBeacons == 0:
                print("no anchor and no beacons, let's drop!")
                backBeacon = [True]
            elif numBeacons > 1:
                dropBeacon = False
            elif dropBeacon:
                print(self.id, "beacon distance limit!")

            # Either drop a beacon in place or drop one behind if we lost comm
            if dropBeacon:
                self.spawnBeacon(True)
            elif all(backBeacon):
                self.spawnBeacon(False)

    def artifactCheck(self):
        # Check the artifact list received from the artifact manager for new artifacts
        for artifact in self.agent.newArtifacts.artifacts:
            artifact_id = str(artifact.position.x) + str(artifact.position.y) + str(artifact.position.z)
            if artifact_id not in self.artifacts:
                self.artifacts[artifact_id] = ArtifactReport(artifact, artifact_id)
                self.report = True
                print(self.id, 'new artifact', artifact_id)

        # If we didn't add anything new, check if any still need reported
        if not self.report:
            for artifact in self.artifacts.values():
                if not artifact.reported:
                    self.report = True
                    break

    def start(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.useSimComms:
                self.simCommCheck()

            for neighbor in self.neighbors.values():
                # TODO make incomm check when the last message was pulled
                neighbor.incomm = neighbor.simcomm

                # Make sure our beacon list matches our neighbors'
                for beacon in neighbor.commBeacons.data:
                    if beacon.active and not self.beacons[beacon.id].active:
                        self.beacons[beacon.id].pos = beacon.pos
                        self.beacons[beacon.id].active = True

            self.publishBeacons()

            # Update the visualization topics
            if self.id in self.monitors:
                self.publishNeighbors()

            if self.type == 'robot':
                self.beaconCheck()
                self.artifactCheck()

                # Change the goal to go home to report
                if self.report:
                    # If we're in comms, assume the current list is reported
                    # Eventually may want to add a confirmation from base
                    if self.neighbors['Anchor'].incomm:
                        self.report = False
                        self.task_pub.publish('Explore')
                        for artifact in self.artifacts.values():
                            artifact.reported = True
                    else:
                        print(self.id, 'return to report...')
                        self.task_pub.publish('Home')

                # Make sure we're not stopped after dropping a beacon
                self.stop.data = False
                self.stop_pub.publish(self.stop)

            rate.sleep()
        return


# TODO fix for launch files, python3 and namespace
if __name__ == '__main__':
    ma = MultiAgent(sys.argv[1], sys.argv[2])

    try:
        ma.start()
    except rospy.ROSInterruptException:
        pass
