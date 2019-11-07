#!/usr/bin/env python
from __future__ import print_function
from cmath import rect, phase
import math
import copy
import hashlib
import rospy

from std_msgs.msg import Bool
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from octomap_msgs.msg import Octomap
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from marble_artifact_detection_msgs.msg import ArtifactArray
from marble_origin_detection_msgs.msg import OriginDetectionStatus
from marble_multi_agent.msg import AgentMsg
from marble_multi_agent.msg import NeighborMsg
from marble_multi_agent.msg import CommsCheckArray
from marble_multi_agent.msg import Beacon
from marble_multi_agent.msg import BeaconArray
from marble_multi_agent.msg import AgentArtifact
from marble_multi_agent.msg import BaseMonitor


class Neighbor(object):
    """ Data structure to hold pertinent information about other agents """

    def __init__(self, agent_id, parent_id):
        self.id = agent_id
        self.pid = parent_id
        self.cid = ''
        self.odometry = Odometry()
        self.goal = PoseStamped()
        self.atnode = Bool()
        self.map = Octomap()
        self.commBeacons = BeaconArray()
        self.newArtifacts = ArtifactArray()
        self.artifacts = {}
        self.lastMessage = rospy.get_rostime()
        self.lastArtifact = ''
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

        # Update the map if it's not empty (ie, high bandwidth message)
        if len(str(neighbor.map.data)) > 100:
            self.map = neighbor.map

        # Update parameters depending on if we're talking directly or not
        if updater:
            self.commBeacons = neighbor.commBeacons
            self.newArtifacts = neighbor.newArtifacts
            self.cid = updater
            self.lastMessage = rospy.get_rostime()
            self.incomm = True
        else:
            self.cid = neighbor.cid
            self.lastMessage = neighbor.lastMessage.data
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


# TODO I don't think we need this anymore, as we're using a separate neighbors array anyway
# Should probably change the Neighbor class to Agent and remove this
class Agent(Neighbor):
    """ Sub-class for an individual agent, with an array of neighbors to pass data """

    def __init__(self, agent_id):
        super(Agent, self).__init__(agent_id, agent_id)
        self.neighbors = {}


class Base(object):
    """ Data structure to hold pertinent information about the base station """

    def __init__(self):
        self.lastMessage = rospy.get_rostime()
        self.lastArtifact = ''
        self.incomm = True
        self.simcomm = True
        self.commBeacons = BeaconArray()


class BeaconObj(object):
    """ Data structure to hold pertinent information about beacons """

    def __init__(self, agent, owner):
        self.id = agent
        self.owner = owner
        self.pos = Point()
        self.simcomm = True
        self.active = False


class ArtifactReport:
    """
    Internal artifact structure to track reporting.
    Holds full artifact message so neighbors can be fused.
    """

    def __init__(self, artifact, artifact_id):
        self.id = artifact_id
        self.artifact = artifact
        self.reported = False
        self.new = True
        self.firstSeen = rospy.get_rostime()


class DataListener:
    """ Listens to all of the applicable topics and repackages into a single object """

    def __init__(self, agent, agent_type, simcomm, topics):
        self.agent = agent  # Agent or Neighbor object

        # self.beacon_sub = \
        #     rospy.Subscriber('/' + self.agent.id + '/beacons',
        #                      BeaconArray, self.Receiver, 'commBeacons', queue_size=1)
        self.artifact_sub = \
            rospy.Subscriber('/' + self.agent.id + '/' + topics['artifacts'],
                             ArtifactArray, self.Receiver, 'newArtifacts', queue_size=1)

        if agent_type == 'robot' or (agent_type == 'base' and simcomm):
            self.odom_sub = \
                rospy.Subscriber('/' + self.agent.id + '/' + topics['odometry'],
                                 Odometry, self.Receiver, 'odometry', queue_size=1)
            self.goal_sub = \
                rospy.Subscriber('/' + self.agent.id + '/' + topics['goal'],
                                 PoseStamped, self.Receiver, 'goal', queue_size=1)
            self.node_sub = \
                rospy.Subscriber('/' + self.agent.id + '/' + topics['node'],
                                 Bool, self.Receiver, 'atnode', queue_size=1)
            self.map_sub = \
                rospy.Subscriber('/' + self.agent.id + '/' + topics['map'],
                                 Octomap, self.Receiver, 'map')

    def Receiver(self, data, parameter):
        # TODO won't need the simcomm check if we're only subscribed to our internal data. ^^^
        if self.agent.simcomm:
            setattr(self.agent, parameter, data)
            self.agent.lastMessage = rospy.get_rostime()

        # Throttle our updates to at least faster than the multi-agent node
        # Don't throttle an octomap or it doesn't update properly
        if parameter != 'map':
            rospy.sleep(0.5)


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


class MultiAgent:
    """ Initialize a multi-agent node for the agent, publishes data for others and listens """

    def __init__(self):
        # Load parameters from the launch file
        self.id = rospy.get_param('multi_agent/vehicle', 'H01')
        self.type = rospy.get_param('multi_agent/type', 'robot')
        # Rate to run the node at
        self.rate = rospy.get_param('multi_agent/rate', 1)
        # Whether to republish neighbor data for visualization or other uses
        self.useMonitor = rospy.get_param('multi_agent/monitor', False)
        # Whether to use simulated comms or real comms
        self.useSimComms = rospy.get_param('multi_agent/simcomms', False)
        # Whether to run the agent without a base station (comms always true)
        self.solo = rospy.get_param('multi_agent/solo', False)
        # Time without message for lost comm
        self.commThreshold = rospy.Duration(rospy.get_param('multi_agent/commThreshold', False), 2)
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
        # Number of beacons this robot is carrying
        self.numBeacons = rospy.get_param('multi_agent/numBeacons', 16)
        # Total number of potential beacons
        totalBeacons = rospy.get_param('multi_agent/totalBeacons', 48)
        # Potential robot neighbors to monitor
        neighbors = rospy.get_param('multi_agent/potentialNeighbors', 'H01,H02,H03').split(',')
        # Topics
        pubLowTopic = rospy.get_param('multi_agent/pubLowTopic', 'low_data')
        pubHighTopic = rospy.get_param('multi_agent/pubHighTopic', 'high_data')
        baseTopic = rospy.get_param('multi_agent/baseTopic', '/Base/ma_status')
        stopTopic = rospy.get_param('multi_agent/stopTopic', 'stop_for_beacon_drop')
        statusTopic = rospy.get_param('multi_agent/statusTopic', 'task_update')
        waitTopic = rospy.get_param('multi_agent/waitTopic', 'origin_detection_status')
        commTopic = rospy.get_param('multi_agent/commTopic', 'base_comm')
        topics = {}
        topics['odometry'] = rospy.get_param('multi_agent/odomTopic', 'odometry')
        topics['goal'] = rospy.get_param('multi_agent/goalTopic', 'node_skeleton/next_turn_pose')
        topics['map'] = rospy.get_param('multi_agent/mapTopic', 'octomap_binary')
        topics['node'] = rospy.get_param('multi_agent/nodeTopic', 'at_node_center')
        topics['artifacts'] = rospy.get_param('multi_agent/artifactsTopic', 'artifact_array/relay')

        # Static anchor position
        self.anchorPos = Point()
        self.anchorPos.x = rospy.get_param('multi_agent/anchorX', 1.0)
        self.anchorPos.y = rospy.get_param('multi_agent/anchorY', 0.0)
        self.anchorPos.z = rospy.get_param('multi_agent/anchorZ', 0.1)

        self.neighbors = {}
        self.beacons = {}
        self.beaconsArray = []
        self.low_sub = {}
        self.high_sub = {}
        self.comm_sub = {}
        self.simcomms = {}
        self.commcheck = {}
        self.artifacts = {}
        self.monitor = {}
        self.history = []
        self.hislen = self.rate * 10  # How long the odometry history should be
        self.minAnchorDist = 10  # Minimum distance before a beacon is ever dropped
        self.report = False
        self.status = ''
        self.wait = True

        # Zero out the beacons if this is a base station type
        if self.type == 'base':
            self.numBeacons = 0

        rospy.init_node(self.id + '_multi_agent')
        self.start_time = rospy.get_rostime()
        while self.start_time.secs == 0:
            self.start_time = rospy.get_rostime()

        # Initialize object for our own data
        self.agent = Agent(self.id)

        # Setup the internal listeners if we're a robot
        if self.type == 'robot':
            DataListener(self.agent, 'robot', self.useSimComms, topics)
            self.task_pub = rospy.Publisher('task', String, queue_size=10)
            self.deploy_pub = rospy.Publisher('deploy', Bool, queue_size=10)
            self.stop_pub = rospy.Publisher(stopTopic, Bool, queue_size=10)
            self.comm_pub = rospy.Publisher(commTopic, Bool, queue_size=10)
            self.wait_sub = rospy.Subscriber(waitTopic, OriginDetectionStatus, self.WaitMonitor)
            self.status_sub = rospy.Subscriber(statusTopic, String, self.StatusMonitor)

        if self.type == 'beacon':
            self.beacon = BeaconObj(self.id, self.id)

        if self.type != 'base':
            # Initialize base station
            self.base = Base()
            self.base_sub = rospy.Subscriber(baseTopic, BaseMonitor, self.BaseMonitor)

        if self.useSimComms:
            self.comm_sub[self.id] = \
                rospy.Subscriber('commcheck', CommsCheckArray, self.simCommChecker, self.id)
            self.comm_sub[self.id] = \
                rospy.Subscriber('/Anchor/commcheck', CommsCheckArray, self.simCommChecker, 'Anchor')

        # Setup the listeners for each neighbor
        for nid in [n for n in neighbors if n != self.id]:
            self.neighbors[nid] = Neighbor(nid, self.id)

            # Subscribers for the packaged data
            subLowTopic = '/' + nid + '/' + pubLowTopic
            subHighTopic = '/' + nid + '/' + pubHighTopic
            self.low_sub[nid] = rospy.Subscriber(subLowTopic, AgentMsg, self.CommReceiver)
            self.high_sub[nid] = rospy.Subscriber(subHighTopic, AgentMsg, self.CommReceiver)

            if self.useSimComms:
                comm_topic = '/' + nid + '/commcheck'
                self.comm_sub[nid] = \
                    rospy.Subscriber(comm_topic, CommsCheckArray, self.simCommChecker, nid)

            # Setup topics for visualization at whichever monitors are specified (always base)
            if self.useMonitor or self.type == 'base':
                topic = 'neighbors/' + nid + '/'
                self.monitor[nid] = {}
                self.monitor[nid]['odometry'] = \
                    rospy.Publisher(topic + 'odometry', Odometry, queue_size=10)
                self.monitor[nid]['goal'] = \
                    rospy.Publisher(topic + 'goal', PoseStamped, queue_size=10)
                self.monitor[nid]['map'] = \
                    rospy.Publisher(topic + 'map', Octomap, queue_size=10)

        # Get our 'number id' for beacon assignment
        if self.type == 'robot':
            try:
                sid = int(self.id[-2:])
            except ValueError:
                sid = int(self.id[-1])
        else:
            sid = 0

        # Setup the beacons.  For real robots the names shouldn't matter as long as consistent
        for i in range(1, totalBeacons + 1):
            nid = 'B' + str(i)

            # Determine if this agent 'owns' the beacon so we don't have conflicting names
            owner = i <= sid * self.numBeacons and i > (sid - 1) * self.numBeacons

            self.beacons[nid] = BeaconObj(nid, owner)

            # Subscribers for the packaged data
            subLowTopic = '/' + nid + '/' + pubLowTopic
            subHighTopic = '/' + nid + '/' + pubHighTopic
            self.low_sub[nid] = rospy.Subscriber(subLowTopic, AgentMsg, self.CommReceiver)
            self.high_sub[nid] = rospy.Subscriber(subHighTopic, AgentMsg, self.CommReceiver)

            if self.useSimComms:
                comm_topic = '/' + nid + '/commcheck'
                self.comm_sub[nid] = \
                    rospy.Subscriber(comm_topic, CommsCheckArray, self.simCommChecker, nid)

        # Publishers for the packaged data.  If we need to use custom publishers for each
        # neighbor we're talking to, then this has to moved above in the neighbor loop
        self.low_pub = rospy.Publisher(pubLowTopic, AgentMsg, queue_size=1)
        self.high_pub = rospy.Publisher(pubHighTopic, AgentMsg, queue_size=1)
        # TODO Don't think this is needed anymore, just put it in the AgentMsg?
        # self.beacon_pub = rospy.Publisher('beacons', BeaconArray, queue_size=10)

        # Setup the base station monitors
        if self.type == 'base':
            # The status message for the agents get latest beacons and artifact acknowledgement
            self.base_pub = rospy.Publisher(baseTopic, BaseMonitor, queue_size=10)

            self.monitor['beacons'] = rospy.Publisher('mbeacons', Marker, queue_size=10)
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

            self.monitor['artifacts'] = rospy.Publisher('martifacts', MarkerArray, queue_size=10)
            self.martifact = MarkerArray()

    def buildArtifactMarkers(self):
        i = 0
        self.martifact.markers = []
        for artifact in self.artifacts.values():

            martifact = Marker()
            martifact.header = artifact.artifact.header
            martifact.header.frame_id = 'world'
            martifact.id = i
            martifact.type = martifact.TEXT_VIEW_FACING
            martifact.action = martifact.ADD

            # Highlight new artifacts, and check if it's now old
            if artifact.new:
                martifact.scale.x = 10.0
                martifact.scale.y = 10.0
                martifact.scale.z = 10.0

                if artifact.firstSeen < rospy.get_rostime() - rospy.Duration(30):
                    artifact.new = False
            else:
                martifact.scale.x = 1.0
                martifact.scale.y = 1.0
                martifact.scale.z = 1.0

            martifact.color.a = 1.0
            martifact.color.r = 1.0
            martifact.color.g = 1.0
            martifact.color.b = 1.0
            martifact.pose.position = artifact.artifact.position
            martifact.text = artifact.artifact.obj_class

            self.martifact.markers.append(martifact)
            i = i + 1

    def publishNeighbors(self):
        # Topics for visualization at the anchor node
        if self.useSimComms:
            for neighbor in self.neighbors.values():
                self.monitor[neighbor.id]['odometry'].publish(neighbor.odometry)
                self.monitor[neighbor.id]['goal'].publish(neighbor.goal)
                self.monitor[neighbor.id]['map'].publish(neighbor.map)

        self.mbeacon.points = []
        for beacon in self.beacons.values():
            if beacon.active:
                self.mbeacon.points.append(beacon.pos)
        self.monitor['beacons'].publish(self.mbeacon)

        self.buildArtifactMarkers()
        self.monitor['artifacts'].publish(self.martifact)

    # TODO Move this to something else since it doesn't actually publish anymore.  Maybe buildAgentMessage?
    def publishBeacons(self):
        commBeacons = []
        for beacon in self.beacons.values():
            if beacon.active:
                commBeacon = Beacon()
                commBeacon.id = beacon.id
                commBeacon.active = beacon.active
                commBeacon.pos = beacon.pos
                commBeacons.append(commBeacon)

        self.beaconsArray = commBeacons
        # self.beacon_pub.publish(self.beaconsArray)

    def WaitMonitor(self, data):
        if data.status > 0:
            self.wait = False

    def StatusMonitor(self, data):
        self.status = data.data

    def BaseMonitor(self, data):
        # Get the beacons from the base, and artifact status
        if self.base.simcomm:
            self.base.lastMessage = rospy.get_rostime()
            self.base.commBeacons.data = data.beacons
            for agent in data.agents:
                if agent.id == self.id:
                    self.base.lastArtifact = agent.lastArtifact
                    break

    def publishBaseMonitor(self):
        # Publish the base's beacon list and artifact status of each neighbor
        msg = BaseMonitor()
        msg.beacons = self.beaconsArray
        for neighbor in self.neighbors.values():
            agent = AgentArtifact()
            agent.id = neighbor.id
            agent.lastArtifact = neighbor.lastArtifact
            msg.agents.append(agent)

        self.base_pub.publish(msg)

    def buildAgentMessage(self, msg, agent, high):
        msg.id = agent.id
        msg.cid = agent.cid
        msg.odometry = agent.odometry
        msg.goal = agent.goal
        msg.lastMessage.data = agent.lastMessage
        msg.newArtifacts = agent.newArtifacts

        # TODO Map will get moved to only self once we start merging maps
        # Only add the map data if we're handling high bandwidth
        if high:
            msg.map = agent.map
        # Data that's only sent via direct comms
        if agent.id == self.id:
            msg.type = self.type
            msg.commBeacons.data = self.beaconsArray
            # TODO this isn't going to work.  Need to figure it out.

    def CommCheck(self):
        # Simply check when the last time we saw a message and set status
        for neighbor in self.neighbors.values():
            neighbor.incomm = neighbor.lastMessage > rospy.get_rostime() - self.commThreshold

        # TODO Do we need this for beacons?!
        # Same with base station
        if self.type == 'robot' and not self.solo:
            self.base.incomm = self.base.lastMessage > rospy.get_rostime() - self.commThreshold

    # Next 3 functions are just for simulated comms.  Otherwise simcomm is True.
    def simCommChecker(self, data, nid):
        # If we're checking our direct comm, set simcomm directly
        if nid == self.id:
            for neighbor in data.data:
                self.simcomms[neighbor.id] = neighbor
        else:
            # Otherwise build our multidimensional checking array
            self.commcheck[nid] = data.data

    def recurCommCheck(self, cid):
        if cid in self.commcheck:
            for check in self.commcheck[cid]:
                if check.incomm and check.id != self.id and not self.simcomms[check.id].incomm:
                    self.simcomms[check.id].incomm = True
                    self.recurCommCheck(check.id)

    def simCommCheck(self):
        # Recursively check who can talk to who
        # TODO removing this enables us to disable comm hopping...might be useful parameter
        for cid in self.commcheck:
            if self.simcomms[cid].incomm:
                self.recurCommCheck(cid)

        # Set the simcomm based on the newly modified matrix
        for simcomm in self.simcomms.values():
            if 'B' in simcomm.id:
                self.beacons[simcomm.id].simcomm = simcomm.incomm
            elif simcomm.id != 'Anchor':
                self.neighbors[simcomm.id].simcomm = simcomm.incomm

        # Base comms are whatever our status with the anchor is
        if self.simcomms and self.id != 'Anchor':
            self.base.simcomm = self.simcomms['Anchor'].incomm

    def CommReceiver(self, data):
        # If I'm a beacon, don't do anything!
        if self.type == 'beacon' and not self.beacon.active:
            activate = False

            # Check the beacon data to see if we should be active yet
            # May not want to do this for real beacons, although could help with data rates
            for beacon in data.commBeacons.data:
                if beacon.id == self.id:
                    activate = True
                    self.beacon.active = True
                    self.beacon.pos = beacon.pos

            if not activate:
                return

        # If we're talking to a beacon, we only update it's neighbor array
        # If we move the Beacons message to the AgentMsg may need to rethink
        runComm = False
        if data.type == 'beacon':
            if self.beacons[data.id].simcomm:
                runComm = True
                notStart = rospy.get_rostime() > rospy.Time(0)
                # Need to figure out how to update commBeacons on self if received from a beacon!
        elif data.type == 'base':
            runComm = True
            notStart = rospy.get_rostime() > rospy.Time(0)
        elif self.neighbors[data.id].simcomm:
            runComm = True
            notStart = self.neighbors[data.id].lastMessage > rospy.Time(0)

            # Load data from our neighbors, and their neighbors
            self.neighbors[data.id].update(data, self.id)

        if runComm:
            # TODO Right now all maps are being sent, even if it's your own.  Can optimize.
            # But, if we just merge before sending, we only need to send one map in the first place.
            # So need to see if that's going to happen before optimizing this
            # We could break the neighbors into multiple publishers so the other agents subscribe
            # to everyone who are not themselves.  That would be more efficient than creating
            # pairs of pubs/subs for each pair!

            # Get our neighbor's neighbors' data and update our own neighbor list
            for neighbor2 in data.neighbors:
                # Make sure the neighbor isn't ourself, it's not a stale message,
                # and we've already talked directly to the neighbor in the last N seconds
                if neighbor2.id != self.id:
                    # TODO need to look at what timestamp we should be looking at for neighbor2
                    # May have just been a problem from restarting mid course
                    newerMessage = (neighbor2.lastMessage.data >
                                    self.neighbors[neighbor2.id].lastMessage + rospy.Duration(5))
                    notDirectComm = self.neighbors[neighbor2.id].cid != self.id
                    incomm = self.neighbors[neighbor2.id].incomm

                    if (notStart and (newerMessage and (notDirectComm or not incomm))):
                        self.neighbors[neighbor2.id].update(neighbor2)

    def updateHistory(self):
        self.history.append(self.agent.odometry.pose.pose)
        if len(self.history) > self.hislen:
            self.history = self.history[-self.hislen:]

    def updateBeacons(self):
        for neighbor in self.neighbors.values():
            # Make sure our beacon list matches our neighbors'
            for beacon in neighbor.commBeacons.data:
                if beacon.active and not self.beacons[beacon.id].active:
                    self.beacons[beacon.id].pos = beacon.pos
                    self.beacons[beacon.id].active = True

        if self.type != 'base':
            for beacon in self.base.commBeacons.data:
                if beacon.active and not self.beacons[beacon.id].active:
                    self.beacons[beacon.id].pos = beacon.pos
                    self.beacons[beacon.id].active = True

    def deployBeacon(self, inplace, dropReason):
        deploy = False
        # Find the first available beacon for this agent
        for beacon in self.beacons.values():
            if beacon.owner and not beacon.active:
                deploy = beacon.id
                break

        if deploy:
            # Stop the robot and publish message to deployment mechanism
            self.stop_pub.publish(True)
            pose = self.agent.odometry.pose.pose

            if self.useSimComms:
                # Either need to identify location before comm loss, or make this a guidance
                # command to return to a point in range
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
                    ret = set_state(state)
                    rospy.sleep(3)
                    print(ret.status_message)
                else:
                    # Wait to stop, send deploy message, then wait for deployment to finish
                    # rospy.sleep(3)
                    self.deploy_pub.publish(True)
                    # rospy.sleep(10)

                # Resume the mission
                self.stop_pub.publish(False)
                # self.deploy_pub.publish(False)

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
                # Update our movement history.  May need to move up one level if we use out of comm
                self.updateHistory()

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
            else:
                # If we're not talking to the base station, attempt to reverse drop
                # self.deployBeacon(False)
                pass

    def baseArtifacts(self):
        for neighbor in self.neighbors.values():
            if neighbor.incomm:
                # Check the artifact list received from the artifact manager for new artifacts
                for artifact in neighbor.newArtifacts.artifacts:
                    artifact_id = (str(artifact.position.x) +
                                   str(artifact.position.y) +
                                   str(artifact.position.z))
                    if artifact_id not in self.artifacts:
                        self.artifacts[artifact_id] = ArtifactReport(artifact, artifact_id)
                        print(self.id, 'new artifact from', neighbor.id, artifact.obj_class, artifact_id)

                artifactString = repr(neighbor.newArtifacts.artifacts).encode('utf-8')
                neighbor.lastArtifact = hashlib.md5(artifactString).hexdigest()

    def artifactCheck(self, agent, artifacts):
        # Check the artifact list received from the artifact manager for new artifacts
        for artifact in agent.newArtifacts.artifacts:
            artifact_id = (str(artifact.position.x) +
                           str(artifact.position.y) +
                           str(artifact.position.z))
            if artifact_id not in artifacts:
                artifacts[artifact_id] = ArtifactReport(artifact, artifact_id)
                print(self.id, 'new artifact from', agent.id, artifact.obj_class, artifact_id)
                if agent.id == self.id:
                    ignore = False
                    for neighbor in self.neighbors.values():
                        for artifact2 in neighbor.artifacts.values():
                            if getDist2D(artifact.position, artifact2.artifact.position) < 3:
                                ignore = True

                    if not ignore:
                        self.report = True

    def artifactCheckReport(self):
        # If we didn't add anything new, check if any still need reported
        if not self.report:
            for artifact in self.artifacts.values():
                if not artifact.reported:
                    self.report = True
                    break

        # Identify our report so we can track that the base station has seen it
        if self.report:
            print('will report...')
            artifactString = repr(self.agent.newArtifacts.artifacts).encode('utf-8')
            self.agent.lastArtifact = hashlib.md5(artifactString).hexdigest()

    def start(self):
        # Wait to start running anything until we've gotten some data and can confirm comms
        # This should also help to recover any beacons being published by other nodes
        # Need to wait for origin detection before we do anything else
        if self.type == 'robot' and not self.useSimComms:
            rospy.sleep(5)
            while self.wait:
                rospy.sleep(1)

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.useSimComms:
                self.simCommCheck()

            # Update incomm based on last message seen
            self.CommCheck()

            # Reconcile beacon list with neighbors', and publish the list
            self.updateBeacons()
            self.publishBeacons()

            # Update the visualization topics and base station status
            if self.type == 'base':
                self.baseArtifacts()
                self.publishBaseMonitor()
                self.publishNeighbors()

            # Non-robot nodes don't need to do the following
            if self.type == 'robot':
                # Update our comm status for anyone who needs it
                self.comm_pub.publish(self.base.incomm)

                # Check if we need to drop a beacon
                self.beaconCheck()

                # Get our neighbors' artifacts so we can deconflict reporting
                for neighbor in self.neighbors.values():
                    self.artifactCheck(neighbor, neighbor.artifacts)

                # Make sure our internal artifact list is up to date, and if we need to report
                self.artifactCheck(self.agent, self.artifacts)
                self.artifactCheckReport()

                # Change the goal to go home to report
                if self.report:
                    # Once we see the base has our latest artifact report we can stop going home
                    if self.solo or self.base.lastArtifact == self.agent.lastArtifact:
                        # Only change our task if we set it!
                        if self.status == 'Report':
                            self.task_pub.publish('Explore')

                        self.report = False
                        for artifact in self.artifacts.values():
                            artifact.reported = True
                    else:
                        print(self.id, 'return to report...')
                        self.task_pub.publish('Report')

            # If I'm a beacon, don't publish anything!  But keep listening for activate message.
            if self.type == 'beacon' and not self.beacon.active:
                rate.sleep()
                continue

            # Publish our data!  Publishing both low and high bandwidth so low doesn't depend
            # on the high bandwidth getting through
            pubDataHigh = AgentMsg()
            self.buildAgentMessage(pubDataHigh, self.agent, True)
            for neighbor in self.neighbors.values():
                msg = NeighborMsg()
                self.buildAgentMessage(msg, neighbor, True)
                pubDataHigh.neighbors.append(msg)

            # Remove the maps from low bandwidth.  May consider removing other data as well.
            pubDataLow = AgentMsg()
            self.buildAgentMessage(pubDataLow, self.agent, False)
            for neighbor in self.neighbors.values():
                msg = NeighborMsg()
                self.buildAgentMessage(msg, neighbor, False)
                pubDataLow.neighbors.append(msg)

            self.low_pub.publish(pubDataLow)
            self.high_pub.publish(pubDataHigh)

            rate.sleep()
        return


if __name__ == '__main__':
    ma = MultiAgent()

    try:
        ma.start()
    except rospy.ROSInterruptException:
        pass
