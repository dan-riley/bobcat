#!/usr/bin/env python
from __future__ import print_function
import hashlib
import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from octomap_msgs.msg import Octomap
from marble_artifact_detection_msgs.msg import ArtifactArray
from marble_multi_agent.msg import AgentMsg
from marble_multi_agent.msg import NeighborMsg
from marble_multi_agent.msg import CommsCheckArray
from marble_multi_agent.msg import Beacon
from marble_multi_agent.msg import BeaconArray
from marble_multi_agent.msg import BaseMonitor
from marble_multi_agent.msg import Goal
from marble_multi_agent.msg import GoalArray
from octomap_merger.msg import OctomapArray


class Agent(object):
    """ Data structure to hold pertinent information about other agents """

    def __init__(self, agent_id, parent_id, agent_type):
        self.id = agent_id
        self.pid = parent_id
        self.cid = ''
        self.type = agent_type
        self.status = ''
        self.guiTaskName = ''
        self.guiTaskValue = ''
        self.guiGoalPoint = PoseStamped()
        self.guiAccept = False
        self.guiGoalAccept = False
        self.odometry = Odometry()
        self.exploreGoal = PoseStamped()
        self.explorePath = Path()
        self.goal = Goal()
        self.goals = GoalArray()
        self.atnode = Bool()
        self.map = Octomap()
        self.mapSize = Float64()
        self.travMap = Octomap()
        self.travMapSize = Float64()
        self.commBeacons = BeaconArray()
        self.newArtifacts = ArtifactArray()
        self.artifacts = {}
        self.lastMessage = rospy.get_rostime()
        self.lastDirectMessage = self.lastMessage
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

    def updateLow(self, neighbor):
        self.status = neighbor.status
        self.odometry = neighbor.odometry
        self.goal = neighbor.goal
        self.newArtifacts = neighbor.newArtifacts

        if neighbor.guiTaskName and neighbor.guiTaskName != self.guiTaskName:
            self.guiTaskName = neighbor.guiTaskName
            self.guiAccept = True
        if neighbor.guiTaskValue and neighbor.guiTaskValue != self.guiTaskValue:
            self.guiTaskValue = neighbor.guiTaskValue
            self.guiAccept = True
        if neighbor.guiGoalPoint.header.frame_id and neighbor.guiGoalPoint != self.guiGoalPoint:
            self.guiGoalPoint = neighbor.guiGoalPoint
            self.guiGoalAccept = True

    def update(self, neighbor, offset, updater=False):
        # Update parameters depending on if we're talking directly or not
        # Ignore most low bandwidth data if it's high bandwidth, since there's a
        # good chance it's older, and may cause jumping info.
        if updater:
            self.commBeacons = neighbor.commBeacons
            self.cid = updater
            self.lastMessage = rospy.get_rostime()
            self.lastDirectMessage = self.lastMessage
            self.incomm = True
            # Update the map if it's not empty (ie, high bandwidth message)
            if neighbor.map.data:
                self.map = neighbor.map
                self.mapSize = neighbor.mapSize
                self.travMap = neighbor.travMap
                self.travMapSize = neighbor.travMapSize
            else:
                self.updateLow(neighbor)
        else:
            # Neighbor data is only low bandwidth
            self.updateLow(neighbor)
            self.cid = neighbor.cid
            self.incomm = False
            # Update the timestamp with the offset between machines
            self.lastMessage = neighbor.lastMessage.data + offset

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


class Base(object):
    """ Data structure to hold pertinent information about the base station """

    def __init__(self):
        self.lastMessage = rospy.get_rostime()
        self.lastArtifact = ''
        self.incomm = True
        self.simcomm = True
        self.commBeacons = BeaconArray()
        self.map = Octomap()
        self.mapSize = Float64()
        self.travMap = Octomap()
        self.travMapSize = Float64()

    def update(self, neighbor):

        # Update the map if it's not empty (ie, high bandwidth message)
        if neighbor.map.data:
            self.map = neighbor.map
            self.mapSize = neighbor.mapSize
            self.travMap = neighbor.travMap
            self.travMapSize = neighbor.travMapSize


class BeaconObj(object):
    """ Data structure to hold pertinent information about beacons """

    def __init__(self, agent, owner):
        self.id = agent
        self.owner = owner
        self.pos = Point()
        self.simcomm = False
        self.active = False
        self.map = Octomap()
        self.mapSize = Float64()
        self.travMap = Octomap()
        self.travMapSize = Float64()

    def update(self, neighbor):

        # Update the map if it's not empty (ie, high bandwidth message)
        if neighbor.map.data:
            self.map = neighbor.map
            self.mapSize = neighbor.mapSize
            self.travMap = neighbor.travMap
            self.travMapSize = neighbor.travMapSize


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

        # Base and beacons need to publish their maps so they can be merged
        self.map_sub = \
            rospy.Subscriber('/' + self.agent.id + '/' + topics['map'],
                             Octomap, self.Receiver, 'map')
        self.size_sub = \
            rospy.Subscriber('/' + self.agent.id + '/' + topics['mapSize'],
                             Float64, self.Receiver, 'mapSize')
        self.trav_map_sub = \
            rospy.Subscriber('/' + self.agent.id + '/' + topics['travMap'],
                             Octomap, self.Receiver, 'travMap')
        self.trav_size_sub = \
            rospy.Subscriber('/' + self.agent.id + '/' + topics['travMapSize'],
                             Float64, self.Receiver, 'travMapSize')

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
        # Time without message for lost comm
        self.commThreshold = rospy.Duration(rospy.get_param('multi_agent/commThreshold', 2))
        # Total number of potential beacons
        totalBeacons = rospy.get_param('multi_agent/totalBeacons', 16)
        # Potential robot neighbors to monitor
        neighbors = rospy.get_param('multi_agent/potentialNeighbors', 'H01,H02,H03').split(',')
        # Beacons this robot is carrying
        myBeacons = rospy.get_param('multi_agent/myBeacons', '').split(',')
        self.numBeacons = len(myBeacons) if myBeacons[0] != '' else 0
        # Topics for publishers
        pubLowTopic = rospy.get_param('multi_agent/pubLowTopic', 'low_data')
        pubHighTopic = rospy.get_param('multi_agent/pubHighTopic', 'high_data')
        self.baseTopic = rospy.get_param('multi_agent/baseTopic', '/Base/ma_status')
        # Topics for subscribers
        topics = {}
        topics['odometry'] = rospy.get_param('multi_agent/odomTopic', 'odometry')
        topics['exploreGoal'] = rospy.get_param('multi_agent/exploreGoalTopic', 'frontier_goal_pose')
        topics['explorePath'] = rospy.get_param('multi_agent/explorePathTopic', 'planned_path')
        topics['goals'] = rospy.get_param('multi_agent/goalsTopic', 'goal_array')
        topics['map'] = rospy.get_param('multi_agent/mapTopic', 'merged_map')
        topics['mapSize'] = rospy.get_param('multi_agent/mapSizeTopic', 'merged_size')
        topics['travMap'] = rospy.get_param('multi_agent/travMapTopic', 'merged_traversability_map')
        topics['travMapSize'] = rospy.get_param('multi_agent/travMapSizeTopic', 'merged_traversability_size')
        topics['node'] = rospy.get_param('multi_agent/nodeTopic', 'at_node_center')
        topics['artifacts'] = rospy.get_param('multi_agent/artifactsTopic', 'artifact_array/relay')

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
        self.wait = False
        self.commListen = False

        rospy.init_node(self.id + '_multi_agent')
        self.start_time = rospy.get_rostime()
        while self.start_time.secs == 0:
            self.start_time = rospy.get_rostime()

        # Initialize object for our own data
        self.agent = Agent(self.id, self.id, self.type)
        DataListener(self.agent, topics)

        if self.type != 'base':
            # Initialize base station
            self.base = Base()
            # TODO get rid of the BaseMonitor
            self.base_sub = rospy.Subscriber(self.baseTopic, BaseMonitor, self.BaseMonitor)
            subLowTopic = '/Base/' + pubLowTopic
            subHighTopic = '/Base/' + pubHighTopic
            self.low_sub['base'] = rospy.Subscriber(subLowTopic, AgentMsg, self.CommReceiver)
            self.high_sub['base'] = rospy.Subscriber(subHighTopic, AgentMsg, self.CommReceiver)

        if self.useSimComms:
            self.comm_sub[self.id] = \
                rospy.Subscriber('commcheck', CommsCheckArray, self.simCommChecker, self.id)

        # Setup the listeners for each neighbor
        for nid in [n for n in neighbors if n != self.id]:
            self.neighbors[nid] = Agent(nid, self.id, 'robot')

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
                self.monitor[nid]['map'] = \
                    rospy.Publisher(topic + 'map', Octomap, queue_size=10)
                self.monitor[nid]['travMap'] = \
                    rospy.Publisher(topic + 'traversability_map', Octomap, queue_size=10)
                self.monitor[nid]['artifacts'] = \
                    rospy.Publisher(topic + 'artifacts', ArtifactArray, queue_size=10)

        # Setup the beacons.  For real robots the names shouldn't matter as long as consistent
        for i in range(1, totalBeacons + 1):
            prefix = '0' if i < 10 else ''
            nid = 'B' + prefix + str(i)

            # Determine if this agent 'owns' the beacon so we don't have conflicting names
            owner = True if nid in myBeacons else False

            self.beacons[nid] = BeaconObj(nid, owner)

            if self.id != nid:
                # Subscribers for the packaged data
                subLowTopic = '/' + nid + '/' + pubLowTopic
                subHighTopic = '/' + nid + '/' + pubHighTopic
                self.low_sub[nid] = rospy.Subscriber(subLowTopic, AgentMsg, self.CommReceiver)
                self.high_sub[nid] = rospy.Subscriber(subHighTopic, AgentMsg, self.CommReceiver)

            if self.useSimComms:
                comm_topic = '/' + nid + '/commcheck'
                self.comm_sub[nid] = \
                    rospy.Subscriber(comm_topic, CommsCheckArray, self.simCommChecker, nid)

        self.merge_pub = rospy.Publisher('neighbor_maps', OctomapArray, queue_size=1)
        self.merge_trav_pub = rospy.Publisher('neighbor_traversability_maps', OctomapArray, queue_size=1)

        # Publishers for the packaged data.  If we need to use custom publishers for each
        # neighbor we're talking to, then this has to moved above in the neighbor loop
        self.low_pub = rospy.Publisher(pubLowTopic, AgentMsg, queue_size=1)
        self.high_pub = rospy.Publisher(pubHighTopic, AgentMsg, queue_size=1)

    def publishMonitors(self):
        for neighbor in self.neighbors.values():
            self.monitor[neighbor.id]['status'].publish(neighbor.status)
            self.monitor[neighbor.id]['incomm'].publish(neighbor.incomm)
            self.monitor[neighbor.id]['odometry'].publish(neighbor.odometry)
            self.monitor[neighbor.id]['goal'].publish(neighbor.goal.pose)
            self.monitor[neighbor.id]['path'].publish(neighbor.goal.path)
            self.monitor[neighbor.id]['map'].publish(neighbor.map)
            self.monitor[neighbor.id]['travMap'].publish(neighbor.travMap)
            self.monitor[neighbor.id]['artifacts'].publish(neighbor.newArtifacts)

    def BaseMonitor(self, data):
        # TODO is this still needed?  Just use the Anchor/low_data message?  add lastArtifact to it
        # Get the beacons from the base, and artifact status
        if self.base.simcomm:
            self.base.lastMessage = rospy.get_rostime()
            self.base.commBeacons.data = data.beacons
            for agent in data.agents:
                if agent.id == self.id:
                    self.base.lastArtifact = agent.lastArtifact
                    break

    def getStatus(self):
        return self.agent.status

    def buildAgentMessage(self, msg, agent, high):
        msg.id = agent.id
        msg.cid = agent.cid
        msg.guiTaskName = agent.guiTaskName
        msg.guiTaskValue = agent.guiTaskValue
        msg.guiGoalPoint = agent.guiGoalPoint
        msg.odometry = agent.odometry
        msg.newArtifacts = agent.newArtifacts
        msg.lastMessage.data = agent.lastMessage
        msg.goal = agent.goal

        # Data that's only sent via direct comms
        if agent.id == self.id:
            msg.status = self.getStatus()
            msg.type = self.type
            msg.commBeacons.data = self.beaconsArray
            # TODO this isn't going to work.  Need to figure it out.
            # Think this is fine if we assume beacons always talk to base?
            msg.mapSize = self.agent.mapSize
            msg.travMapSize = self.agent.travMapSize
            # Only add the map data if we're handling high bandwidth
            if high:
                msg.map = agent.map
                msg.travMap = agent.travMap
        else:
            msg.status = agent.status

    def CommCheck(self):
        # Simply check when the last time we saw a message and set status
        for neighbor in self.neighbors.values():
            neighbor.incomm = neighbor.lastDirectMessage > rospy.get_rostime() - self.commThreshold

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
        # Wait until everything is initialized before processing any data
        if not self.commListen:
            return

        # Approximately account for different system times.  Assumes negligible transmit time.
        offset = rospy.get_rostime() - data.header.stamp

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
                notStart = rospy.get_rostime() > rospy.Time(0)
                # Need to figure out how to update commBeacons on self if received from a beacon!
                # TODO if we assume all beacons are talking to base then this isn't needed?
                self.beacons[data.id].update(data)
        elif data.type == 'base':
            if self.base.simcomm:
                runComm = True
                notStart = self.base.lastMessage > rospy.Time(0)
                self.base.update(data)
        elif self.neighbors[data.id].simcomm:
            runComm = True
            notStart = self.neighbors[data.id].lastMessage > rospy.Time(0)

            # Verify that the neighbor received this goal point then clear it out
            if (data.guiTaskName and data.guiTaskValue and
                    (self.neighbors[data.id].guiTaskName == data.guiTaskName and
                     self.neighbors[data.id].guiTaskValue == data.guiTaskValue)):
                self.neighbors[data.id].guiTaskName = ''
                self.neighbors[data.id].guiTaskValue = ''
            if (data.guiGoalPoint.header.frame_id and
                    self.neighbors[data.id].guiGoalPoint == data.guiGoalPoint):
                self.neighbors[data.id].guiGoalPoint = PoseStamped()

            # Don't accept the GUI commands unless here or risk overwriting
            data.guiTaskName = ''
            data.guiTaskValue = ''
            data.guiGoalPoint = PoseStamped()

            # Load data from our neighbors, and their neighbors
            self.neighbors[data.id].update(data, offset, self.id)

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
                    # If the message is coming from the same place, take all messages so we don't
                    # miss a high bandwidth message.  Otherwise add an offset to prevent looping.
                    if data.id == neighbor2.cid:
                        messOffset = rospy.Duration(0)
                    else:
                        messOffset = self.commThreshold

                    newerMessage = (neighbor2.lastMessage.data + offset >
                                    self.neighbors[neighbor2.id].lastMessage + messOffset)

                    notDirectComm = self.neighbors[neighbor2.id].cid != self.id
                    incomm = self.neighbors[neighbor2.id].incomm

                    if (notStart and (newerMessage and (notDirectComm or not incomm))):
                        # Don't accept the GUI commands transmitted from the agents at the base
                        if self.type == 'base':
                            neighbor2.guiTaskName = ''
                            neighbor2.guiTaskValue = ''
                            neighbor2.guiGoalPoint = PoseStamped()
                        self.neighbors[neighbor2.id].update(neighbor2, offset)

                elif neighbor2.lastMessage.data + offset + rospy.Duration(1) > self.base.lastMessage:
                    # Accept the GUI commands coming from a neighbor if its new and not empty
                    if (neighbor2.guiTaskName and neighbor2.guiTaskValue and
                            (self.agent.guiTaskName != neighbor2.guiTaskName or
                             self.agent.guiTaskValue != neighbor2.guiTaskValue)):
                        self.agent.guiTaskName = neighbor2.guiTaskName
                        self.agent.guiTaskValue = neighbor2.guiTaskValue
                        self.agent.guiAccept = True
                    if (neighbor2.guiGoalPoint.header.frame_id and
                            self.agent.guiGoalPoint != neighbor2.guiGoalPoint):
                        self.agent.guiGoalPoint = neighbor2.guiGoalPoint
                        self.agent.guiGoalAccept = True

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

    def run(self):
        return False

    def start(self):
        # Wait to start running anything until we've gotten some data and can confirm comms
        # This should also help to recover any beacons being published by other nodes
        # Need to wait for origin detection before we do anything else
        if self.type == 'robot' and not self.useSimComms:
            rospy.sleep(5)
            # while self.wait:
            #     rospy.sleep(1)

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.useSimComms:
                self.simCommCheck()

            # Update incomm based on last message seen
            self.CommCheck()

            # Reconcile beacon list with neighbors'
            self.updateBeacons()

            if self.useMonitor:
                self.publishMonitors()

            # Execute the type-specific functions
            if not self.run():
                # If run returns False (usually for an inactive beacon), skip rest of the function
                rate.sleep()
                continue

            # Publish the neighbor maps so merge node can merge with our map
            mergeMaps = OctomapArray()
            mergeMaps.num_octomaps = 0
            for neighbor in self.neighbors.values():
                if neighbor.map.data:
                    mergeMaps.octomaps.append(neighbor.map)
                    mergeMaps.owners.append(neighbor.id)
                    mergeMaps.sizes.append(neighbor.mapSize.data)
                    mergeMaps.num_octomaps += 1

            for beacon in self.beacons.values():
                if beacon.map.data:
                    mergeMaps.octomaps.append(beacon.map)
                    mergeMaps.owners.append(beacon.id)
                    mergeMaps.sizes.append(beacon.mapSize.data)
                    mergeMaps.num_octomaps += 1

            if hasattr(self, 'base') and self.base.map.data:
                mergeMaps.octomaps.append(self.base.map)
                mergeMaps.owners.append('Base')
                mergeMaps.sizes.append(self.base.mapSize.data)
                mergeMaps.num_octomaps += 1

            mergeMaps.header.stamp = rospy.get_rostime()
            self.merge_pub.publish(mergeMaps)

            mergeTravMaps = OctomapArray()
            mergeTravMaps.num_octomaps = 0
            for neighbor in self.neighbors.values():
                if neighbor.travMap.data:
                    mergeTravMaps.octomaps.append(neighbor.travMap)
                    mergeTravMaps.owners.append(neighbor.id)
                    mergeTravMaps.sizes.append(neighbor.travMapSize.data)
                    mergeTravMaps.num_octomaps += 1

            for beacon in self.beacons.values():
                if beacon.travMap.data:
                    mergeTravMaps.octomaps.append(beacon.travMap)
                    mergeTravMaps.owners.append(beacon.id)
                    mergeTravMaps.sizes.append(beacon.travMapSize.data)
                    mergeTravMaps.num_octomaps += 1

            if hasattr(self, 'base') and self.base.travMap.data:
                mergeTravMaps.octomaps.append(self.base.travMap)
                mergeTravMaps.owners.append('Base')
                mergeTravMaps.sizes.append(self.base.travMapSize.data)
                mergeTravMaps.num_octomaps += 1

            mergeTravMaps.header.stamp = rospy.get_rostime()
            self.merge_trav_pub.publish(mergeTravMaps)

            # Publish our data!  Publishing both low and high bandwidth so low doesn't depend
            # on the high bandwidth getting through
            # Not including neighbor data in high bandwidth to ease conflicts
            pubDataHigh = AgentMsg()
            self.buildAgentMessage(pubDataHigh, self.agent, True)

            # Remove the maps from low bandwidth.  May consider removing other data as well.
            pubDataLow = AgentMsg()
            self.buildAgentMessage(pubDataLow, self.agent, False)
            for neighbor in self.neighbors.values():
                msg = NeighborMsg()
                self.buildAgentMessage(msg, neighbor, False)
                pubDataLow.neighbors.append(msg)

            pubDataLow.header.stamp = rospy.get_rostime()
            self.low_pub.publish(pubDataLow)
            pubDataHigh.header.stamp = rospy.get_rostime()
            self.high_pub.publish(pubDataHigh)

            rate.sleep()
        return
