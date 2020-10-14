#!/usr/bin/env python
from __future__ import print_function
import math
import hashlib
import rospy
import copy

from std_msgs.msg import Bool
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from marble_artifact_detection_msgs.msg import ArtifactArray
from marble_artifact_detection_msgs.msg import ArtifactImg
from marble_multi_agent.msg import AgentMsg
from marble_multi_agent.msg import AgentReset
from marble_multi_agent.msg import NeighborMsg
from marble_multi_agent.msg import CommsCheckArray
from marble_multi_agent.msg import Beacon
from marble_multi_agent.msg import BeaconArray
from marble_multi_agent.msg import Goal
from marble_multi_agent.msg import GoalArray
from marble_multi_agent.msg import DMReq
from marble_multi_agent.msg import DMReqArray
from marble_multi_agent.msg import DMResp
from marble_multi_agent.msg import DMRespArray
from marble_mapping.msg import OctomapArray
from marble_mapping.msg import OctomapNeighbors


def getDist(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)


def getDist2D(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)


class Agent(object):
    """ Data structure to hold pertinent information about other agents """

    def __init__(self, agent_id, parent_id, agent_type, report_images):
        self.id = agent_id
        self.pid = parent_id
        self.cid = ''
        self.type = agent_type
        self.reset = AgentReset()
        self.reportImages = report_images
        self.lastMessage = rospy.get_rostime()
        self.lastDirectMessage = self.lastMessage
        self.incomm = True
        self.simcomm = True
        self.initialize()
        self.initializeMaps()

    def initialize(self, resetTime=rospy.Time()):
        self.status = ''
        self.guiStamp = rospy.get_rostime()
        self.guiTaskName = ''
        self.guiTaskValue = ''
        self.guiGoalPoint = PoseStamped()
        self.guiAccept = False
        self.guiGoalAccept = False
        self.odometry = Odometry()
        self.exploreGoal = PoseStamped()
        self.explorePath = Path()
        self.exploreGoal.header.frame_id = 'world'
        self.explorePath.header.frame_id = 'world'
        self.goal = Goal()
        self.goals = GoalArray()
        self.atnode = Bool()
        self.commBeacons = BeaconArray()
        self.newArtifacts = ArtifactArray()
        self.checkArtifacts = ArtifactArray()
        self.images = []
        self.missingImages = []
        self.lastArtifact = ''
        self.resetStamp = resetTime
        if resetTime:
            self.resetAgent = True
        else:
            self.resetAgent = False

    def initializeMaps(self, numDiffs=0, diffClear=False):
        self.mapDiffs = OctomapArray()
        self.mapDiffs.owner = self.id
        self.updateMapDiffs = False
        self.numDiffs = numDiffs
        self.missingDiffs = []
        self.diffClear = diffClear

    def updateCommon(self, neighbor):
        self.status = neighbor.status
        self.odometry = neighbor.odometry
        self.goal = neighbor.goal
        self.newArtifacts = neighbor.newArtifacts

        # Update missing diffs if the neighbor said there are new ones
        if not self.diffClear and neighbor.numDiffs > self.numDiffs and not self.reset.ignore:
            for i in range(self.numDiffs, neighbor.numDiffs):
                if i not in self.missingDiffs:
                    self.missingDiffs.append(i)
            self.numDiffs = neighbor.numDiffs

        # Identify new images available for request
        for artifact in neighbor.newArtifacts.artifacts:
            if (artifact.image_data.format != 'empty' and
                    artifact.artifact_id not in self.images and
                    artifact.artifact_id not in self.missingImages):
                # Images we know about so we don't re-mark them for download
                self.images.append(artifact.artifact_id)
                # Images we haven't received yet
                self.missingImages.append(artifact.artifact_id)

        # Ignore the GUI message if it's old
        if neighbor.guiStamp.data > self.guiStamp:
            self.guiUpdate(neighbor)

    def update(self, neighbor, updater=False):
        # If this agent has been reset, don't accept messages until confirmed it has reset
        if self.resetAgent:
            if neighbor.reset.stamp < self.resetStamp:
                return
            else:
                self.resetAgent = False

        # Update parameters depending on if we're talking directly or not
        if updater:
            self.commBeacons = neighbor.commBeacons
            self.cid = updater
            self.lastMessage = neighbor.header.stamp
            self.lastDirectMessage = rospy.get_rostime()
            self.incomm = True
            self.updateCommon(neighbor)
        else:
            self.updateCommon(neighbor)
            self.cid = neighbor.cid
            self.incomm = False
            self.lastMessage = neighbor.lastMessage.data

    def guiUpdate(self, neighbor):
        self.guiStamp = neighbor.guiStamp.data

        if neighbor.guiTaskName and neighbor.guiTaskValue:
            self.guiTaskName = neighbor.guiTaskName
            self.guiTaskValue = neighbor.guiTaskValue
            self.guiAccept = True

        # Accept goal point if it's updated
        if neighbor.guiGoalPoint.header.seq > self.guiGoalPoint.header.seq:
            self.guiGoalPoint = neighbor.guiGoalPoint

        # Only accept reset if it's newer than the last one for this agent
        if neighbor.reset.stamp > self.reset.stamp:
            self.reset = neighbor.reset

    def addArtifact(self, artifact):
        self.checkArtifacts.artifacts.append(artifact)
        self.checkArtifacts.owner = self.id
        self.checkArtifacts.num_artifacts += 1

    def updateHash(self):
        # If set, ignore images when checking if Base has received new artifacts
        if not self.reportImages:
            for artifact in self.checkArtifacts.artifacts:
                artifact.image_data.data = []
        artifactString = repr(self.checkArtifacts.artifacts).encode('utf-8')
        self.lastArtifact = hashlib.md5(artifactString).hexdigest()


class Base(object):
    """ Data structure to hold pertinent information about the base station """

    def __init__(self):
        self.baseStamp = rospy.get_rostime()
        self.lastMessage = rospy.get_rostime()
        self.lastDirectMessage = self.lastMessage
        self.lastArtifact = ''
        self.incomm = True
        self.simcomm = True
        self.baseArtifacts = []
        self.commBeacons = BeaconArray()

    def update(self, neighbor):
        self.lastMessage = neighbor.header.stamp
        self.lastDirectMessage = rospy.get_rostime()
        self.commBeacons = neighbor.commBeacons

    def updateArtifacts(self, agent_id, neighbor):
        self.baseStamp = neighbor.baseStamp.data
        self.baseArtifacts = neighbor.baseArtifacts
        for agent in neighbor.baseArtifacts:
            if agent.id == agent_id:
                self.lastArtifact = agent.lastArtifact
                break

    def resetArtifact(self, agent_id):
        for agent in self.baseArtifacts:
            if agent.id == agent_id:
                self.lastArtifact = ''
                break


class BeaconObj(object):
    """ Data structure to hold pertinent information about beacons """

    def __init__(self, agent, owner):
        self.id = agent
        self.owner = owner
        self.pos = Point()
        self.lastMessage = rospy.get_rostime()
        self.lastDirectMessage = self.lastMessage
        self.incomm = False
        self.simcomm = False
        self.active = False

    def update(self, neighbor):
        self.lastMessage = neighbor.header.stamp
        self.lastDirectMessage = rospy.get_rostime()


class ArtifactReport:
    """
    Internal artifact structure to track reporting.
    Holds full artifact message so neighbors can be fused.
    """

    def __init__(self, agent_id, artifact, sendImages):
        self.id = artifact.artifact_id
        self.agent_id = agent_id
        self.artifact = artifact
        self.reported = False
        self.score = 0
        self.new = True
        self.firstSeen = rospy.get_rostime()
        self.lastPublished = rospy.Time()
        self.originals = {}

        # Mark empty data so we receiver doesn't try to request it
        if not artifact.image_data.data or not sendImages:
            artifact.image_data.format = 'empty'
        # Save image so we can send it via DM
        self.image = ArtifactImg()
        if sendImages:
            self.image.artifact_id = artifact.artifact_id
            self.image.artifact_img = artifact.image_data


class DataListener:
    """ Listens to all of the applicable topics and repackages into a single object """

    def __init__(self, agent, topics):
        self.agent = agent  # Agent object

        if self.agent.type == 'robot':
            self.artifact_sub = \
                rospy.Subscriber('/' + self.agent.id + '/' + topics['artifacts'],
                                 ArtifactArray, self.Receiver, 'newArtifacts', queue_size=1)
            self.odom_sub = \
                rospy.Subscriber('/' + self.agent.id + '/' + topics['odometry'],
                                 Odometry, self.Receiver, 'odometry', queue_size=1)
            self.explore_goal_sub = \
                rospy.Subscriber('/' + self.agent.id + '/' + topics['exploreGoal'],
                                 PoseStamped, self.Receiver, 'exploreGoal', queue_size=1)
            self.explore_path_sub = \
                rospy.Subscriber('/' + self.agent.id + '/' + topics['explorePath'],
                                 Path, self.Receiver, 'explorePath', queue_size=1)
            self.goals_sub = \
                rospy.Subscriber('/' + self.agent.id + '/' + topics['goals'],
                                 GoalArray, self.Receiver, 'goals', queue_size=1)
            self.node_sub = \
                rospy.Subscriber('/' + self.agent.id + '/' + topics['node'],
                                 Bool, self.Receiver, 'atnode', queue_size=1)
            self.mapdiffs_sub = \
                rospy.Subscriber('/' + self.agent.id + '/' + topics['mapDiffs'],
                                 OctomapArray, self.Receiver, 'mapDiffs')

    def Receiver(self, data, parameter):
        setattr(self.agent, parameter, data)


class MultiAgent(object):
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
        # Whether to include images in reports at all (disable for low bandwidth!)
        self.sendImages = rospy.get_param('multi_agent/sendImages', True)
        # Whether to ensure images are sent with artifacts to decide whether reporting is complete
        self.reportImages = rospy.get_param('multi_agent/reportImages', True)
        # Time without message for lost comm
        self.commThreshold = rospy.Duration(rospy.get_param('multi_agent/commThreshold', 2))
        # Time to wait before trying another agent for direct message requests
        self.dmWait = rospy.Duration(rospy.get_param('multi_agent/dmWait', 3))
        # Whether to send DMs in one large message or split for comms
        self.dmSplit = rospy.Duration(rospy.get_param('multi_agent/dmSplit', True))
        # Total number of potential beacons
        totalBeacons = rospy.get_param('multi_agent/totalBeacons', 16)
        # Potential robot neighbors to monitor
        neighbors = rospy.get_param('multi_agent/potentialNeighbors', 'H01,H02,H03').split(',')
        # Beacons this robot is carrying
        self.myBeacons = rospy.get_param('multi_agent/myBeacons', '').split(',')
        self.numBeacons = len(self.myBeacons) if self.myBeacons[0] != '' else 0
        # Topics for publishers
        self.pubTopic = rospy.get_param('multi_agent/pubTopic', 'ma_data')
        self.commTopic = rospy.get_param('multi_agent/commTopic', 'mesh_comm')
        useMesh = rospy.get_param('multi_agent/useMesh', False)
        self.useVirtual = rospy.get_param('multi_agent/useVirtual', False)
        # Topics for subscribers
        topics = {}
        topics['odometry'] = rospy.get_param('multi_agent/odomTopic', 'odometry')
        topics['exploreGoal'] = rospy.get_param('multi_agent/exploreGoalTopic', 'frontier_goal_pose')
        topics['explorePath'] = rospy.get_param('multi_agent/explorePathTopic', 'planned_path')
        topics['goals'] = rospy.get_param('multi_agent/goalsTopic', 'goal_array')
        topics['mapDiffs'] = rospy.get_param('multi_agent/mapDiffsTopic', 'map_diffs')
        topics['node'] = rospy.get_param('multi_agent/nodeTopic', 'at_node_center')
        topics['artifacts'] = rospy.get_param('multi_agent/artifactsTopic', 'artifact_array/relay')
        topics['artifactImages'] = rospy.get_param('multi_agent/artifactImagesTopic', 'artifact_image_to_base')

        if not self.sendImages:
            self.reportImages = False

        self.neighbors = {}
        self.beacons = {}
        self.beaconsArray = []
        self.data_sub = {}
        self.comm_sub = {}
        self.dmReq_pub = {}
        self.dmReq_sub = {}
        self.dmResp_pub = {}
        self.dmResp_sub = {}
        self.simcomms = {}
        self.commcheck = {}
        self.artifacts = {}
        self.monitor = {}
        self.wait = False  # Change to True to wait for Origin Detection
        self.commListen = False
        self.artifactsUpdated = False
        self.lastDMReq = rospy.Time()
        self.dmReqs = []

        rospy.init_node(self.id + '_multi_agent')
        self.start_time = rospy.get_rostime()
        while self.start_time.secs == 0:
            self.start_time = rospy.get_rostime()

        # Initialize object for our own data
        self.agent = Agent(self.id, self.id, self.type, self.reportImages)
        DataListener(self.agent, topics)

        # Initialize base station
        self.base = Base()

        if useMesh:
            # UDP Mesh broadcast publishes to one topic but subscribes to different
            self.broadcastPubTopic = self.commTopic + '/' + self.pubTopic
            self.broadcastSubTopic = self.commTopic + '/' + self.id + '/' + self.pubTopic
        elif self.useVirtual:
            # Virtual uses a send/recv prefix, and Sub is set in setupComms
            self.broadcastPubTopic = self.commTopic + '/send/' + self.pubTopic
        else:
            # Multimaster and sim use same topic for publishing and subscribing
            self.broadcastPubTopic = self.commTopic + '/' + self.pubTopic
            self.broadcastSubTopic = self.commTopic + '/' + self.pubTopic

        if self.type != 'base':
            self.setupComms('Base')

        if self.useSimComms:
            self.comm_sub[self.id] = \
                rospy.Subscriber('commcheck', CommsCheckArray, self.simCommChecker, self.id)

        # Setup the listeners for each neighbor
        for nid in [n for n in neighbors if n != self.id]:
            if nid:
                self.addNeighbor(nid, 'robot')

        # Setup the beacons.  For real robots the names shouldn't matter as long as consistent
        for i in range(1, totalBeacons + 1):
            prefix = '0' if i < 10 else ''
            nid = 'B' + prefix + str(i)

            if self.id != nid:
                self.addNeighbor(nid, 'beacon')
            else:
                self.beacons[nid] = BeaconObj(nid, False)

        self.neighbor_maps_pub = rospy.Publisher('neighbor_maps', OctomapNeighbors, latch=True, queue_size=1)

        # Publisher for the packaged data
        self.data_pub = rospy.Publisher(self.broadcastPubTopic, AgentMsg, queue_size=1)

    def addNeighbor(self, nid, agent_type):
        if agent_type == 'robot':
            self.neighbors[nid] = Agent(nid, self.id, agent_type, self.reportImages)
        else:
            # Determine if this agent 'owns' the beacon so we don't have conflicting names
            owner = True if nid in self.myBeacons else False
            self.beacons[nid] = BeaconObj(nid, owner)

        # Beacons don't run a node in virtual, so don't setup comms
        if agent_type != 'beacon' or (agent_type == 'beacon' and not self.useVirtual):
            self.setupComms(nid)

        if self.useSimComms:
            comm_topic = '/' + nid + '/commcheck'
            self.comm_sub[nid] = \
                rospy.Subscriber(comm_topic, CommsCheckArray, self.simCommChecker, nid)

        # Setup topics for visualization at whichever monitors are specified (always base)
        if agent_type == 'robot' and (self.useMonitor or self.type == 'base'):
            topic = 'neighbors/' + nid + '/'
            self.monitor[nid] = {}
            self.monitor[nid]['status'] = \
                rospy.Publisher(topic + 'status', String, queue_size=10)
            self.monitor[nid]['incomm'] = \
                rospy.Publisher(topic + 'incomm', Bool, queue_size=10)
            self.monitor[nid]['odometry'] = \
                rospy.Publisher(topic + 'odometry', Odometry, queue_size=10)
            self.monitor[nid]['goal'] = \
                rospy.Publisher(topic + 'goal', PoseStamped, queue_size=10)
            self.monitor[nid]['path'] = \
                rospy.Publisher(topic + 'path', Path, queue_size=10)
            self.monitor[nid]['artifacts'] = \
                rospy.Publisher(topic + 'artifacts', ArtifactArray, queue_size=10)
            self.monitor[nid]['guiTaskNameReceived'] = \
                rospy.Publisher(topic + 'guiTaskNameReceived', String, queue_size=10)
            self.monitor[nid]['guiTaskValueReceived'] = \
                rospy.Publisher(topic + 'guiTaskValueReceived', String, queue_size=10)
            self.monitor[nid]['image'] = \
                rospy.Publisher(topic + 'image', ArtifactImg, queue_size=10, latch=True)

    def setupComms(self, nid):
        if self.useVirtual:
            subTopic = self.commTopic + '/recv/' + nid + '/' + self.pubTopic
            pubDMReqTopic = self.commTopic + '/send/' + nid + '/dm_request'
            subDMReqTopic = self.commTopic + '/recv/' + nid + '/dm_request'
            pubDMRespTopic = self.commTopic + '/send/' + nid + '/dm_response'
            subDMRespTopic = self.commTopic + '/recv/' + nid + '/dm_response'
        else:
            subTopic = '/' + nid + '/' + self.broadcastSubTopic
            pubDMReqTopic = '/' + self.id + '/' + self.commTopic + '/' + nid + '/dm_request'
            subDMReqTopic = '/' + nid + '/' + self.commTopic + '/' + self.id + '/dm_request'
            pubDMRespTopic = '/' + self.id + '/' + self.commTopic + '/' + nid + '/dm_response'
            subDMRespTopic = '/' + nid + '/' + self.commTopic + '/' + self.id + '/dm_response'

        # Subscribers for the packaged data
        self.data_sub[nid] = rospy.Subscriber(subTopic, AgentMsg, self.CommReceiver)

        # Pairs for direct message requests
        self.dmReq_pub[nid] = rospy.Publisher(pubDMReqTopic, DMReqArray, queue_size=1)
        self.dmReq_sub[nid] = rospy.Subscriber(subDMReqTopic, DMReqArray, self.DMRequestReceiever, nid)

        # Pairs for direct message responses
        self.dmResp_pub[nid] = rospy.Publisher(pubDMRespTopic, DMRespArray, queue_size=1)
        self.dmResp_sub[nid] = rospy.Subscriber(subDMRespTopic, DMRespArray, self.DMResponseReceiever, nid)

    def publishMonitors(self):
        for neighbor in self.neighbors.values():
            self.monitor[neighbor.id]['status'].publish(neighbor.status)
            self.monitor[neighbor.id]['incomm'].publish(neighbor.incomm)
            self.monitor[neighbor.id]['guiTaskNameReceived'].publish(neighbor.guiTaskName)
            self.monitor[neighbor.id]['guiTaskValueReceived'].publish(neighbor.guiTaskValue)
            # Don't publish if the robot hasn't initialized odometry
            if (neighbor.odometry.pose.pose.position.x != 0 and
                neighbor.odometry.pose.pose.position.y != 0):
                self.monitor[neighbor.id]['odometry'].publish(neighbor.odometry)
                self.monitor[neighbor.id]['goal'].publish(neighbor.goal.pose)
                self.monitor[neighbor.id]['path'].publish(neighbor.goal.path)
                self.monitor[neighbor.id]['artifacts'].publish(neighbor.checkArtifacts)
        for artifact in self.artifacts.values():
            if artifact.image.artifact_img.data and rospy.get_rostime() - artifact.lastPublished > rospy.Duration(60) and artifact.agent_id != self.id:
                self.monitor[artifact.agent_id]['image'].publish(artifact.image)
                artifact.lastPublished = rospy.get_rostime()

    def getStatus(self):
        return self.agent.status

    def subsample(self, goal):
        pubgoal = Goal()
        pubgoal.pose = goal.pose
        pubgoal.path.header.frame_id = goal.path.header.frame_id

        for i, pose in enumerate(goal.path.poses):
            if i % 20 == 0:
                pubgoal.path.poses.append(pose)

        if pubgoal.path.poses and pubgoal.path.poses[-1] != goal.path.poses[-1]:
            pubgoal.path.poses.append(goal.path.poses[-1])

        return pubgoal

    def buildAgentMessage(self, msg, agent):
        msg.id = agent.id
        msg.cid = agent.cid
        msg.guiStamp.data = agent.guiStamp
        msg.guiTaskName = agent.guiTaskName
        msg.guiTaskValue = agent.guiTaskValue
        msg.guiGoalPoint = agent.guiGoalPoint
        msg.odometry = agent.odometry
        msg.lastMessage.data = agent.lastMessage
        msg.goal = self.subsample(agent.goal)
        msg.reset = agent.reset

        # Need to clear any image data.  Will be overwritten by subscriber if done elswhere
        msg.newArtifacts = copy.deepcopy(agent.checkArtifacts)
        for artifact in msg.newArtifacts.artifacts:
            artifact.image_data.data = []

        # Data that's only sent via direct comms
        if agent.id == self.id:
            msg.status = self.getStatus()
            msg.type = self.type
            msg.baseStamp.data = self.base.baseStamp
            msg.baseArtifacts = self.base.baseArtifacts
            msg.commBeacons.data = self.beaconsArray
            msg.numDiffs = agent.mapDiffs.num_octomaps
        else:
            msg.status = agent.status
            msg.numDiffs = agent.numDiffs

    def CommCheck(self):
        if rospy.get_rostime() < self.start_time + self.commThreshold:
            return

        checkTime = rospy.get_rostime() - self.commThreshold
        # Simply check when the last time we saw a message and set status
        for neighbor in self.neighbors.values():
            neighbor.incomm = neighbor.lastDirectMessage > checkTime

        # Same for beacons
        for beacon in self.beacons.values():
            beacon.incomm = beacon.lastDirectMessage > checkTime

        # Same with base station
        if self.type != 'base' and not self.solo:
            self.base.incomm = self.base.lastDirectMessage > checkTime

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
            if cid in self.simcomms and self.simcomms[cid].incomm:
                self.recurCommCheck(cid)

        # Set the simcomm based on the newly modified matrix
        for simcomm in self.simcomms.values():
            if 'B' in simcomm.id and simcomm.id in self.beacons:
                self.beacons[simcomm.id].simcomm = simcomm.incomm
            elif simcomm.id != 'Base' and simcomm.id in self.neighbors:
                self.neighbors[simcomm.id].simcomm = simcomm.incomm

        # Base comms are whatever our status with the anchor is
        if self.simcomms and self.id != 'Base':
            self.base.simcomm = self.simcomms['Base'].incomm

    def beaconCommCheck(self, data):
        return True

    def CommReceiver(self, data):
        # Wait until the entire sub-object is initialized before processing any data
        if not self.commListen:
            return

        # If I'm a beacon, don't do anything with the data unless activated!
        if self.type == 'beacon':
            if not self.beaconCommCheck(data):
                return

        # If we're talking to a beacon, we only update it's neighbor array
        # If we move the Beacons message to the AgentMsg may need to rethink
        runComm = False
        if data.type == 'beacon':
            if self.beacons[data.id].simcomm:
                runComm = True
                # Need to figure out how to update commBeacons on self if received from a beacon!
                # TODO if we assume all beacons are talking to base then this isn't needed?
                self.beacons[data.id].update(data)
        elif data.type == 'base':
            if self.base.simcomm:
                runComm = True
                self.base.update(data)
                self.base.updateArtifacts(self.id, data)
        elif self.neighbors[data.id].simcomm:
            runComm = True

            # Load data from our neighbors, and their neighbors
            self.neighbors[data.id].update(data, self.id)

            # Update the base station artifacts list if it's newer from this neighbor
            if data.baseStamp.data > self.base.baseStamp:
                self.base.updateArtifacts(self.id, data)

        if runComm:
            # Get our neighbor's neighbors' data and update our own neighbor list
            for neighbor2 in data.neighbors:
                # Make sure the neighbor isn't ourself, it's not a stale message,
                # and we've already talked directly to the neighbor in the last N seconds
                if neighbor2.id != self.id:
                    # Dynamically add neighbors we don't know about
                    if neighbor2.id not in self.neighbors:
                        self.addNeighbor(neighbor2.id, 'robot')

                    newerMessage = (neighbor2.lastMessage.data >
                                    self.neighbors[neighbor2.id].lastMessage)

                    notDirectComm = self.neighbors[neighbor2.id].cid != self.id
                    incomm = self.neighbors[neighbor2.id].incomm

                    if (newerMessage and (notDirectComm or not incomm)):
                        self.neighbors[neighbor2.id].update(neighbor2)
                    elif neighbor2.guiStamp.data > self.neighbors[neighbor2.id].guiStamp:
                        # Accept the GUI commands coming from a neighbor if its new and not empty
                        self.neighbors[neighbor2.id].guiUpdate(neighbor2)

                elif neighbor2.guiStamp.data > self.agent.guiStamp:
                    # Accept the GUI commands coming from a neighbor if its new and not empty
                    self.agent.guiUpdate(neighbor2)

    def resetDataCheck(self, data):
        # ma_reset clears all data from the neighbor and artifacts, except map data
        # Clear removes all existing diffs, but keeps sequence the same
        # Reset removes all existing diffs, and resets the sequence to 0, fetching reported diffs
        # Ignore stops listening to this agent's map (does not clear!) until ignore set false
        # Hard Reset restarts the map and ma data like fresh start, on all agents including target
        # Sequences in seqs removes only that sequence and will keep skipping unless reset called

        # Check if the flag for this type of agent is set
        applyAgent = False
        if ((self.type == 'base' and data.base) or
            (self.type == 'robot' and data.robots) or self.type == 'beacon'):
            applyAgent = True

        nid = data.agent
        if nid and applyAgent and data.stamp > self.neighbors[nid].resetStamp:
            rospy.loginfo(self.id + ' resetting data for ' + nid)
            self.neighbors[nid].resetStamp = data.stamp
            self.neighbors[nid].diffClear = data.clear

            if data.ma_reset:
                # Reset the agent's multiagent data
                self.neighbors[nid].initialize(data.stamp)
                self.base.resetArtifact(nid)
                # Remove this agent's artifacts from our list
                for key in [key for key in self.artifacts if self.artifacts[key].agent_id == nid]:
                    del self.artifacts[key]

            if data.clear or data.reset or data.hardReset:
                if data.clear:
                    numDiffs = self.neighbors[nid].numDiffs
                else:
                    numDiffs = 0
                self.neighbors[nid].initializeMaps(numDiffs, True)
            else:
                # If we got a sequence then remove just these from our local map
                for seq in data.seqs:
                    self.neighbors[nid].diffClear = True
                    remove = None
                    for idx, mapDiff in enumerate(self.neighbors[nid].mapDiffs.octomaps):
                        if mapDiff.header.seq == seq:
                            remove = idx
                            break
                    if remove != None:
                        del self.neighbors[nid].mapDiffs.octomaps[remove]

            # Save the data.  Have to do it here in case we did ma_reset
            self.neighbors[nid].guiStamp = rospy.get_rostime()
            self.neighbors[nid].reset = data

    def hardResetCheck(self):
        # Reset self map and multiagent data (if passed)
        reset = self.agent.reset
        if reset.stamp > self.agent.resetStamp and reset.agent == self.id and reset.hardReset:
            rospy.loginfo(self.id + ' hard resetting!')
            # If ma_reset true then re-initialize everything
            if reset.ma_reset:
                self.agent.initialize(reset.stamp)
                self.base.resetArtifact(self.id)
                self.artifacts = {}
                self.reset_pub.publish(True)
                for neighbor in self.neighbors.values():
                    neighbor.initialize()
            else:
                self.agent.resetStamp = reset.stamp

            # Clear out the maps
            self.agent.initializeMaps(0, True)
            for neighbor in self.neighbors.values():
                neighbor.initializeMaps(0, True)

            return True

        return False

    def addMapDiffs(self, nid, nresp, agent):
        nresp.mapDiffs.owner = agent.id
        nresp.mapDiffs.num_octomaps = 0

        if agent.id == self.id:
            mapDiffs = self.agent.mapDiffs.octomaps
        else:
            mapDiffs = self.neighbors[agent.id].mapDiffs.octomaps

        # Add each requested diff to the message
        for i in agent.missingDiffs:
            for mapDiff in mapDiffs:
                if mapDiff.header.seq == i:
                    nresp.mapDiffs.octomaps.append(mapDiff)
                    nresp.mapDiffs.num_octomaps += 1
                    if self.dmSplit:
                        # If splitting responses, publish then reset the response message
                        self.dmResp_pub[nid].publish([nresp])
                        nresp = DMResp()
                        nresp.id = agent.id
                        nresp.mapDiffs.owner = agent.id
                        nresp.mapDiffs.num_octomaps = 0
                    break

    def addImages(self, nid, nresp, agent):
        # Add each requested image to the message
        for i in agent.missingImages:
            for artifact in self.artifacts.values():
                if artifact.image.artifact_id == i and artifact.image.artifact_img.data:
                    nresp.images.append(artifact.image)
                    if self.dmSplit:
                        # If splitting responses, publish then reset the response message
                        self.dmResp_pub[nid].publish([nresp])
                        nresp = DMResp()
                        nresp.id = agent.id
                    break

    def DMRequestReceiever(self, req, nid):
        # TODO add a time check so we don't try to send again if we already sent recently,
        # as the comms client may be trying to take care of the resend
        resp = []
        for agent in req.agents:
            nresp = DMResp()
            nresp.id = agent.id

            # We might receive a request for an agent we didn't know about before so add neighbor
            if agent.id != self.id and agent.id not in self.neighbors:
                self.addNeighbor(agent.id, 'robot')

            self.addMapDiffs(nid, nresp, agent)
            self.addImages(nid, nresp, agent)
            if not self.dmSplit:
                resp.append(nresp)

        if not self.dmSplit:
            self.dmResp_pub[nid].publish(resp)

    def DMResponseReceiever(self, resp, nid):
        receivedDM = False
        for agent in resp.agents:
            neighbor = self.neighbors[agent.id]
            # Add the new diffs to our array and update the total
            for octomap in agent.mapDiffs.octomaps:
                neighbor.mapDiffs.octomaps.append(octomap)
                neighbor.mapDiffs.num_octomaps += 1
                neighbor.updateMapDiffs = True
                # Remove the received diffs, in case we didn't get all of them
                if octomap.header.seq in neighbor.missingDiffs:
                    neighbor.missingDiffs.remove(octomap.header.seq)
                receivedDM = True

            # Add the new images to our artifacts
            for image in agent.images:
                for artifact in self.artifacts.values():
                    if artifact.artifact.artifact_id == image.artifact_id:
                        artifact.image = image
                        # Add the image to the checkArtifact so we can update the hash table
                        if self.reportImages:
                            for checkArtifact in neighbor.checkArtifacts.artifacts:
                                if checkArtifact.artifact_id == image.artifact_id:
                                    checkArtifact.image_data = image.artifact_img
                                    neighbor.updateHash()
                                    self.artifactsUpdated = True
                                    break

                        # Remove the received image, in case we didn't get all of them
                        if image.artifact_id in neighbor.missingImages:
                            neighbor.missingImages.remove(image.artifact_id)
                        receivedDM = True
                        break

        # Clear out the request log so we don't skip any
        if receivedDM:
            self.lastDMReq = rospy.get_rostime() - self.dmWait
            self.dmReqs = []

    def requestMissing(self):
        # If we've made a request recently, give it some time to try someone else
        if rospy.get_rostime() - self.lastDMReq < self.dmWait:
            return

        # Build a full request list of all missing diffs
        requestFrom = False
        reqs = DMReqArray()
        for neighbor in self.neighbors.values():
            # This neighbors' request
            req = DMReq()
            req.id = neighbor.id
            addRequest = False

            # Look for missing maps and add to request
            if neighbor.missingDiffs:
                req.missingDiffs = neighbor.missingDiffs
                addRequest = True

            # Look for missing images and add to request
            if neighbor.missingImages:
                req.missingImages = neighbor.missingImages
                addRequest = True

            if addRequest:
                reqs.agents.append(req)
                # Request directly from the first one we have missing maps from
                if not requestFrom and neighbor.incomm:
                    requestFrom = neighbor.id

        # If we have any, find someone to request from
        if reqs.agents:
            # Try the base station first; stationary presumed more reliable!
            if not requestFrom and self.id != 'Base' and self.base.incomm and 'Base' not in self.dmReqs:
                requestFrom = 'Base'

            # Try beacons next
            if not requestFrom:
                for beacon in self.beacons.values():
                    if self.id != beacon.id and beacon.incomm and beacon.id not in self.dmReqs:
                        requestFrom = beacon.id
                        break

            # Finally try robots
            if not requestFrom:
                for neighbor in self.neighbors.values():
                    if neighbor.incomm and neighbor.id not in self.dmReqs:
                        requestFrom = neighbor.id
                        break

            # Publish the request to this agent
            if requestFrom:
                self.lastDMReq = rospy.get_rostime()
                self.dmReqs.append(requestFrom)
                self.dmReq_pub[requestFrom].publish(reqs)
            else:
                # If we're missing a map but don't have anyone to request from, start over
                self.lastDMReq = rospy.get_rostime() - self.dmWait
                self.dmReqs = []

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

        # Update the beacons array that gets published with all known active beacons
        commBeacons = []
        for beacon in self.beacons.values():
            if beacon.active:
                commBeacon = Beacon()
                commBeacon.id = beacon.id
                commBeacon.active = beacon.active
                commBeacon.pos = beacon.pos
                commBeacons.append(commBeacon)

        self.beaconsArray = commBeacons

    def artifactCheck(self, agent):
        updateString = False
        # Check the artifact list received from the artifact manager for new artifacts
        for artifact in agent.newArtifacts.artifacts:
            aid = artifact.artifact_id
            if aid not in self.artifacts and artifact.position.x and artifact.position.y:
                addArtifact = True
                # Check if there's a similar artifact already stored so we don't report
                if agent.id == self.id:
                    # Mark that we need to update our hash
                    updateString = False
                    ignore = False
                    for artifact2 in self.artifacts.values():
                        if getDist2D(artifact.position, artifact2.artifact.position) < 3:
                            ignore = True

                    # Skip artifacts that might be another robot
                    if artifact.obj_class == 'rope':
                        for neighbor in self.neighbors.values():
                            if getDist(neighbor.odometry.pose.pose.position, artifact.position) < 5:
                                addArtifact = False
                                ignore = True
                                rospy.loginfo(self.id + ' skipping artifact due to neighbor')
                                break

                    if not ignore:
                        self.report = True
                        updateString = True

                # Now add the artifact to the array
                self.artifacts[aid] = ArtifactReport(agent.id, artifact, self.sendImages)

                if addArtifact:
                    agent.addArtifact(artifact)
                else:
                    self.artifacts[aid].reported = True

                # At the base station, fuse the artifacts and make sure we mark an update
                if self.type == 'base':
                    updateString = True
                    self.fuseArtifact(self.artifacts[aid])

                rospy.loginfo(self.id + ' new artifact from ' + agent.id + ' ' + artifact.obj_class + ' ' + aid)

        if updateString:
            agent.updateHash()
            return True

        return False

    def updateArtifacts(self):
        for neighbor in self.neighbors.values():
            updatedArtifacts = self.artifactCheck(neighbor)

            if updatedArtifacts:
                self.artifactsUpdated = True

    def run(self):
        return False

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

            # Reconcile beacon list with neighbors'
            self.updateBeacons()

            # Request any missing data from each agent
            self.requestMissing()

            # Execute the type-specific functions
            if not self.run():
                # If run returns False (usually for an inactive beacon), skip rest of the function
                rate.sleep()
                continue

            # Check if we need to hard reset map and multiagent
            hardReset = self.hardResetCheck()

            # Build the data message for self and neighbors
            pubData = AgentMsg()
            self.buildAgentMessage(pubData, self.agent)
            neighbor_diffs = OctomapNeighbors()
            pubMapDiffs = False
            for neighbor in self.neighbors.values():
                # Check this neighbor to see if anything should be reset
                self.resetDataCheck(neighbor.reset)
                msg = NeighborMsg()
                self.buildAgentMessage(msg, neighbor)
                pubData.neighbors.append(msg)

                # Get all of the map diffs to publish for the merger
                neighbor_diffs.neighbors.append(neighbor.mapDiffs)
                neighbor_diffs.num_neighbors += 1

                # Only publish if we have new diffs or we've removed some
                if neighbor.updateMapDiffs or neighbor.diffClear:
                    neighbor.updateMapDiffs = False
                    pubMapDiffs = True

                    if neighbor.diffClear:
                        neighbor_diffs.clear = True
                        neighbor.diffClear = False

            pubData.header.stamp = rospy.get_rostime()
            self.data_pub.publish(pubData)
            if pubMapDiffs or hardReset:
                if hardReset:
                    # Only pass hardReset for resetting self map!
                    neighbor_diffs.hardReset = True
                    neighbor_diffs.clear = True
                self.neighbor_maps_pub.publish(neighbor_diffs)

            if self.useMonitor:
                self.publishMonitors()

            rate.sleep()
        return
