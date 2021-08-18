#!/usr/bin/env python
from __future__ import print_function
import rospy
import copy
import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from marble_artifact_detection_msgs.msg import ArtifactArray
from marble_artifact_detection_msgs.msg import ArtifactImg
from bobcat.msg import AgentMsg
from bobcat.msg import NeighborMsg
from bobcat.msg import CommsCheckArray
from bobcat.msg import Beacon
from bobcat.msg import GoalArray
from bobcat.msg import PointArray
from bobcat.msg import PointArrays
from bobcat.msg import DMReq
from bobcat.msg import DMReqArray
from bobcat.msg import DMResp
from bobcat.msg import DMRespArray
from marble_mapping.msg import OctomapArray
from marble_mapping.msg import OctomapNeighbors

from util.helpers import getDist, getDist2D, getAngle, getSeq
from containers import Agent, Base, BeaconObj, ArtifactReport


class BOBCAT(object):
    """ Initialize a multi-agent node for the agent, publishes data for others and listens """

    def __init__(self):
        # Load parameters from the launch file
        self.id = rospy.get_param('bobcat/vehicle', 'H01')
        self.type = rospy.get_param('bobcat/type', 'robot')
        # Rate to run the node at
        self.rate = rospy.get_param('bobcat/rate', 1)
        # Whether to republish neighbor data for visualization or other uses
        self.useViz = rospy.get_param('bobcat/viz', False)
        # Whether to use simulated comms or real comms
        self.useSimComms = rospy.get_param('bobcat/simcomms', False)
        # Whether to run the agent without a base station (comms always true)
        self.solo = rospy.get_param('bobcat/solo', False)
        # Whether we're sharing our pose graph
        self.sharePoseGraph = rospy.get_param('bobcat/sharePoseGraph', False)
        # Whether to include images in reports at all (disable for low bandwidth!)
        self.sendImages = rospy.get_param('bobcat/sendImages', True)
        # Whether to ensure images are sent with artifacts to decide whether reporting is complete
        self.reportImages = rospy.get_param('bobcat/reportImages', True)
        # Time without message for lost comm
        self.commThreshold = rospy.Duration(rospy.get_param('bobcat/commThreshold', 2))
        # Time to wait before trying another agent for direct message requests
        self.dmWait = rospy.Duration(rospy.get_param('bobcat/dmWait', 3))
        # Whether to send DMs in one large message or split for comms
        self.dmSplit = rospy.Duration(rospy.get_param('bobcat/dmSplit', True))
        # Total number of potential beacons
        totalBeacons = rospy.get_param('bobcat/totalBeacons', 16)
        # Potential robot neighbors to monitor
        neighbors = rospy.get_param('bobcat/potentialNeighbors', 'H01,H02,H03').split(',')
        # Beacons this robot is carrying
        self.myBeacons = rospy.get_param('bobcat/myBeacons', '').split(',')
        self.numBeacons = len(self.myBeacons) if self.myBeacons[0] != '' else 0
        self.smartBeacons = rospy.get_param('bobcat/smartBeacons', True)
        # Subsampling parameters for various paths.  Set distance to 0 for no subsampling.
        self.ssDistanceGoalPath = rospy.get_param('bobcat/subsampleDistanceGoalPath', 0)
        self.ssAngleGoalPath = rospy.get_param('bobcat/subsampleAngleGoalPath', 0)
        self.ssDistancePoseGraph = rospy.get_param('bobcat/subsampleDistancePoseGraph', 0)
        self.ssAnglePoseGraph = rospy.get_param('bobcat/subsampleAnglePoseGraph', 0)
        # Topics for publishers
        self.pubTopic = rospy.get_param('bobcat/pubTopic', 'ma_data')
        self.commTopic = rospy.get_param('bobcat/commTopic', 'mesh_comm')
        useMesh = rospy.get_param('bobcat/useMesh', False)
        self.useVirtual = rospy.get_param('bobcat/useVirtual', False)
        # Topics for subscribers
        topics = {}
        topics['odometry'] = rospy.get_param('bobcat/odomTopic', 'odometry')
        topics['exploreGoal'] = rospy.get_param('bobcat/exploreGoalTopic', 'frontier_goal_pose')
        topics['explorePath'] = rospy.get_param('bobcat/explorePathTopic', 'planned_path')
        topics['goals'] = rospy.get_param('bobcat/goalsTopic', 'goal_array')
        topics['mapDiffs'] = rospy.get_param('bobcat/mapDiffsTopic', 'map_diffs')
        topics['node'] = rospy.get_param('bobcat/nodeTopic', 'at_node_center')
        topics['artifacts'] = rospy.get_param('bobcat/artifactsTopic', 'artifact_array/relay')
        topics['poseGraph'] = rospy.get_param('bobcat/poseGraphTopic', 'lio_sam/mapping/path')

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
        self.viz = {}
        self.wait = False  # Change to True to wait for Origin Detection
        self.commListen = False
        self.artifactsUpdated = False
        self.updateMapDiffsArray = False
        self.updatePoseGraphArray = False
        self.lastDMReq = rospy.Time()
        self.dmReqs = []

        rospy.init_node(self.id + '_bobcat')
        self.start_time = rospy.get_rostime()
        while self.start_time.secs == 0:
            self.start_time = rospy.get_rostime()

        # Initialize object for our own data
        self.agent = Agent(self.id, self.id, self.type, self.reportImages)
        self.DataListener(topics)

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

        if self.sharePoseGraph:
            self.lastPoseGraphTime = rospy.Time()
            # Use a TF buffer server to reduce cpu usage
            self.tf_client = tf2_ros.BufferClient('tf2_buffer_server')
            self.tf_client.wait_for_server()

            # Pose Graph republisher after subsample and transform
            self.pose_graph_pub = rospy.Publisher('pose_graph_subsampled', Path, latch=True, queue_size=1)
            # Array of pose graphs for planner deconfliction
            self.pose_graph_array_pub = rospy.Publisher('pose_graph_array', PointArrays, latch=True, queue_size=1)

    ##### Start Local Message Aggregation #####
    def DataListener(self, topics):
        """ Listens to all of the applicable topics and repackages into a single object """

        if self.type == 'robot':
            self.artifact_sub = \
                rospy.Subscriber('/' + self.id + '/' + topics['artifacts'],
                                 ArtifactArray, self.Receiver, 'newArtifacts', queue_size=1)
            self.odom_sub = \
                rospy.Subscriber('/' + self.id + '/' + topics['odometry'],
                                 Odometry, self.Receiver, 'odometry', queue_size=1)
            self.explore_goal_sub = \
                rospy.Subscriber('/' + self.id + '/' + topics['exploreGoal'],
                                 PoseStamped, self.Receiver, 'exploreGoal', queue_size=1)
            self.explore_path_sub = \
                rospy.Subscriber('/' + self.id + '/' + topics['explorePath'],
                                 Path, self.Receiver, 'explorePath', queue_size=1)
            self.goals_sub = \
                rospy.Subscriber('/' + self.id + '/' + topics['goals'],
                                 GoalArray, self.Receiver, 'goals', queue_size=1)
            self.node_sub = \
                rospy.Subscriber('/' + self.id + '/' + topics['node'],
                                 Bool, self.Receiver, 'atnode', queue_size=1)
            self.mapdiffs_sub = \
                rospy.Subscriber('/' + self.id + '/' + topics['mapDiffs'],
                                 OctomapArray, self.Receiver, 'mapDiffs')
            self.posegraph_sub = \
                rospy.Subscriber('/' + self.id + '/' + topics['poseGraph'],
                                 Path, self.Receiver, 'poseGraph')

    def Receiver(self, data, parameter):
        setattr(self.agent, parameter, data)

        # If we're not receiving a goal, set the end of the path as the goal
        if parameter == 'explorePath' and data.poses:
            if data.poses[-1].pose.position != self.agent.exploreGoal.pose.position:
                self.agent.exploreGoal = data.poses[-1]

    ##### Stop Local Message Aggregation #####

    ##### Start Remote Message Aggregation #####
    def addNeighbor(self, nid, agent_type):
        if agent_type == 'robot':
            self.neighbors[nid] = Agent(nid, self.id, agent_type, self.reportImages)
        else:
            # Determine if this agent 'owns' the beacon so we don't have conflicting names
            owner = True if nid in self.myBeacons else False
            self.beacons[nid] = BeaconObj(nid, owner)

        # Only setup comms with beacons if smart beacons enabled
        if agent_type != 'beacon' or (agent_type == 'beacon' and self.smartBeacons):
            self.setupComms(nid)

        if self.useSimComms:
            comm_topic = '/' + nid + '/commcheck'
            self.comm_sub[nid] = \
                rospy.Subscriber(comm_topic, CommsCheckArray, self.simCommChecker, nid)

        # Setup topics for visualization when toggled in launch (always base)
        if agent_type == 'robot' and (self.useViz or self.type == 'base'):
            topic = 'neighbors/' + nid + '/'
            self.viz[nid] = {}
            self.viz[nid]['status'] = \
                rospy.Publisher(topic + 'status', String, queue_size=10)
            self.viz[nid]['incomm'] = \
                rospy.Publisher(topic + 'incomm', Bool, queue_size=10)
            self.viz[nid]['battery'] = \
                rospy.Publisher(topic + 'battery', Float32, queue_size=10)
            self.viz[nid]['odometry'] = \
                rospy.Publisher(topic + 'odometry', Odometry, queue_size=10)
            self.viz[nid]['goal'] = \
                rospy.Publisher(topic + 'goal', PoseStamped, queue_size=10)
            self.viz[nid]['path'] = \
                rospy.Publisher(topic + 'path', Path, queue_size=10)
            self.viz[nid]['poseGraph'] = \
                rospy.Publisher(topic + 'pose_graph', Path, queue_size=10)
            self.viz[nid]['artifacts'] = \
                rospy.Publisher(topic + 'artifacts', ArtifactArray, queue_size=10)
            self.viz[nid]['guiTaskNameReceived'] = \
                rospy.Publisher(topic + 'guiTaskNameReceived', String, queue_size=10)
            self.viz[nid]['guiTaskValueReceived'] = \
                rospy.Publisher(topic + 'guiTaskValueReceived', String, queue_size=10)
            self.viz[nid]['image'] = \
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
    ##### Stop Remote Message Aggregation #####

    ##### Start Output Aggregation #####
    def publishViz(self):
        for neighbor in self.neighbors.values():
            self.viz[neighbor.id]['status'].publish(neighbor.status)
            self.viz[neighbor.id]['incomm'].publish(neighbor.incomm)
            self.viz[neighbor.id]['battery'].publish(neighbor.battery)
            self.viz[neighbor.id]['guiTaskNameReceived'].publish(neighbor.guiTaskName)
            self.viz[neighbor.id]['guiTaskValueReceived'].publish(neighbor.guiTaskValue)
            # Don't publish if the robot hasn't initialized odometry
            if (neighbor.odometry.pose.pose.position.x != 0 and
                neighbor.odometry.pose.pose.position.y != 0):
                self.viz[neighbor.id]['odometry'].publish(neighbor.odometry)
                self.viz[neighbor.id]['goal'].publish(neighbor.goal.pose)
                self.viz[neighbor.id]['path'].publish(neighbor.goal.path)
                self.viz[neighbor.id]['poseGraph'].publish(neighbor.poseGraph)
                self.viz[neighbor.id]['artifacts'].publish(neighbor.checkArtifacts)
        for artifact in self.artifacts.values():
            if artifact.image.artifact_img.data and rospy.get_rostime() - artifact.lastPublished > rospy.Duration(60) and artifact.agent_id != self.id:
                self.viz[artifact.agent_id]['image'].publish(artifact.image)
                artifact.lastPublished = rospy.get_rostime()

    def getStatus(self):
        return self.agent.status

    def buildAgentMessage(self, msg, agent):
        msg.id = agent.id
        msg.cid = agent.cid
        msg.battery = agent.battery
        msg.guiStamp.data = agent.guiStamp
        msg.guiTaskName = agent.guiTaskName
        msg.guiTaskValue = agent.guiTaskValue
        msg.guiGoalPoint = agent.guiGoalPoint
        msg.latestPoseGraph = agent.latestPoseGraph
        msg.lastMessage.data = agent.lastMessage
        msg.reset = agent.reset

        # Extract just the pose for odometry
        msg.odometry.header = agent.odometry.header
        msg.odometry.pose = agent.odometry.pose.pose

        # Only publish artifacts if we have a new one, or three times every 30 seconds
        curtime = rospy.get_rostime()
        if (self.agent.lastArtifact != self.base.lastArtifact or
                curtime > agent.lastArtifactPub + rospy.Duration(30)):
            if curtime > agent.lastArtifactPub + rospy.Duration(33):
                agent.lastArtifactPub = curtime;
            msg.newArtifacts = copy.deepcopy(agent.checkArtifacts)
            # Need to clear any image data.  Will be overwritten by subscriber if done elswhere
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
            # Build the goal message.  Message type is compressed so have to assign separately.
            msg.goal.cost = agent.goal.cost
            msg.goal.path = self.compressPath(agent.goal.path, self.ssDistanceGoalPath, self.ssAngleGoalPath, False)
        else:
            msg.status = agent.status
            msg.numDiffs = agent.numDiffs
            msg.goal = agent.goalCompressed

    def transformPose(self, pose):
        try:
            pose.header.stamp = rospy.Time(0)
            return self.tf_client.transform(pose, "world", rospy.Duration(1))
        except tf2_ros.LookupException, tf2_ros.ExtrapolationException:
            return

    def addPoseToPaths(self, pose, path, cpath):
        # Pose is not mutable so won't be changed.  Path and cpath are changed/returned.
        if pose.header.frame_id != 'world':
            pose = self.transformPose(pose)
        path.poses.append(pose)
        cpath.append(10 * pose.pose.position.x)
        cpath.append(10 * pose.pose.position.y)
        cpath.append(10 * pose.pose.position.z)

    def compressPath(self, path, distance, angle, poseGraph):
        # Subsample a path based on distance/angle, transform to world, and compress
        cpath = []
        if len(path.poses) == 0:
            return cpath

        pubpath = Path()
        pubpath.header.frame_id = 'world'

        # Get the first pose and add to our data
        pivot = path.poses[0]
        self.addPoseToPaths(pivot, pubpath, cpath)
        lastp = path.poses[0].pose.position

        for pose in path.poses:
            nextp = pose.pose.position
            # Make sure the point is at least min distance, and there's been a change in direction
            if (getDist(lastp, pivot.pose.position) > distance):
                if (getAngle(lastp, pivot.pose.position, nextp) > angle):
                    self.addPoseToPaths(pivot, pubpath, cpath)
                    lastp = pivot.pose.position

            pivot = pose

        # Make sure the end point is in the path
        # Need to transform first if it's not since pubpath.poses are
        if pivot.header.frame_id != 'world':
            pivot = self.transformPose(pivot)
        if pubpath.poses[-1] != pivot:
            self.addPoseToPaths(pivot, pubpath, cpath)

        if poseGraph:
            self.agent.latestPoseGraph += 1
            pubpath.header.seq = self.agent.latestPoseGraph
            self.pose_graph_pub.publish(pubpath)

        return cpath
    ##### Stop Output Aggregation #####

    ##### Start Communications Handling #####
    # Comms Monitor for all agents
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
    ##### Stop Communications Handling #####

    ##### Start Message Deconfliction #####
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

                    # We might have gotten empty artifacts on previous versions of this
                    if not newerMessage:
                        newerMessage = (neighbor2.lastMessage.data ==
                                        self.neighbors[neighbor2.id].lastMessage and
                                        neighbor2.newArtifacts.artifacts)

                    if (newerMessage and (notDirectComm or not incomm)):
                        self.neighbors[neighbor2.id].update(neighbor2)
                    elif neighbor2.guiStamp.data > self.neighbors[neighbor2.id].guiStamp:
                        # Accept the GUI commands coming from a neighbor if its new and not empty
                        self.neighbors[neighbor2.id].guiUpdate(neighbor2)

                elif neighbor2.guiStamp.data > self.agent.guiStamp:
                    # Accept the GUI commands coming from a neighbor if its new and not empty
                    self.agent.guiUpdate(neighbor2)
    ##### Stop Message Deconfliction #####

    ##### Start Direct Message Handling #####
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

    def addPoseGraph(self, nid, nresp, agent):
        if agent.missingPoseGraph:
            # Find the right pose graph or return if we don't have it
            if agent.id == self.id:
                poseGraph = self.agent.poseGraphCompressed
                latestPoseGraph = self.agent.latestPoseGraph
            elif self.neighbors[agent.id].latestPoseGraphAvailable >= agent.missingPoseGraph:
                poseGraph = self.neighbors[agent.id].poseGraphCompressed
                latestPoseGraph = self.neighbors[agent.id].latestPoseGraphAvailable
            else:
                return

            nresp.poseGraph = poseGraph
            nresp.latestPoseGraph = latestPoseGraph
            if self.dmSplit:
                self.dmResp_pub[nid].publish([nresp])

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
            self.addPoseGraph(nid, nresp, agent)
            if not self.dmSplit:
                resp.append(nresp)

        if not self.dmSplit:
            self.dmResp_pub[nid].publish(resp)

    def DMResponseReceiever(self, resp, nid):
        receivedDM = False
        for agent in resp.agents:
            neighbor = self.neighbors[agent.id]
            resort = False
            # Add the new diffs to our array and update the total
            for octomap in agent.mapDiffs.octomaps:
                neighbor.mapDiffs.octomaps.append(octomap)
                neighbor.mapDiffs.num_octomaps += 1
                neighbor.updateMapDiffs = True
                self.updateMapDiffsArray = True
                # Remove the received diffs, in case we didn't get all of them
                if octomap.header.seq in neighbor.missingDiffs:
                    neighbor.missingDiffs.remove(octomap.header.seq)
                # We got a diff out of order, so make sure we re-sort the array
                if octomap.header.seq < neighbor.numDiffs - 1:
                    resort = True
                receivedDM = True

            if resort:
                neighbor.mapDiffs.octomaps.sort(key=getSeq)

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

            if agent.poseGraph and agent.latestPoseGraph > neighbor.latestPoseGraphAvailable:
                neighbor.poseGraphCompressed = agent.poseGraph
                neighbor.poseGraph = neighbor.decompressPath(agent.poseGraph)
                neighbor.latestPoseGraphAvailable = agent.latestPoseGraph
                neighbor.missingPoseGraph = False
                self.updatePoseGraphArray = True
                receivedDM = True

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

            if neighbor.latestPoseGraph > neighbor.latestPoseGraphAvailable:
                req.missingPoseGraph = neighbor.latestPoseGraphAvailable + 1
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
    ##### Stop Direct Message Handling #####

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

                        for beacon in self.beacons.values():
                            if getDist(beacon.pos, artifact.position) < 2:
                                addArtifact = False
                                ignore = True
                                rospy.loginfo(self.id + ' skipping artifact due to beacon')
                                break

                    if not ignore:
                        # Add to the artifact count since last report and if it's
                        # the first of a new set of artifacts, start the timer for reporting
                        if self.numNewArtifacts == 0:
                            self.newArtifactTime = rospy.get_rostime()
                        self.numNewArtifacts += 1
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

    ##### BOBCAT Execution #####
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
            pgarray = PointArrays()
            pubMapDiffs = False
            for neighbor in self.neighbors.values():
                # Check this neighbor to see if anything should be reset
                self.resetDataCheck(neighbor.reset)
                msg = NeighborMsg()
                self.buildAgentMessage(msg, neighbor)
                pubData.neighbors.append(msg)

                # Only build the array if we have an update
                if self.updateMapDiffsArray or neighbor.diffClear:
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

                if self.sharePoseGraph and self.updatePoseGraphArray:
                    points = PointArray()
                    for pose in neighbor.poseGraph.poses:
                        points.points.append(pose.pose.position)
                    pgarray.arrays.append(points)

            # Regular multi-agent data
            pubData.header.stamp = rospy.get_rostime()
            self.data_pub.publish(pubData)

            # Neighbor maps for local map merging
            if pubMapDiffs or hardReset:
                if hardReset:
                    # Only pass hardReset for resetting self map!
                    neighbor_diffs.hardReset = True
                    neighbor_diffs.clear = True
                self.updateMapDiffsArray = False
                neighbor_diffs.header.stamp = rospy.get_rostime()
                self.neighbor_maps_pub.publish(neighbor_diffs)

            # Pose graph array for local planner
            if self.sharePoseGraph and self.updatePoseGraphArray:
                self.updatePoseGraphArray = False
                self.pose_graph_array_pub.publish(pgarray)

            # Visualizations
            if self.useViz:
                self.publishViz()

            rate.sleep()
        return
