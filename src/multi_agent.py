#!/usr/bin/env python
from __future__ import print_function
import math
import rospy

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
        self.atnode = Bool()
        self.map = MarkerArray()
        self.commBeacons = BeaconArray()
        self.newArtifacts = ArtifactArray()
        self.lastMessage = rospy.get_rostime()
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


class Base(object):
    """ Data structure to hold pertinent information about the base station """

    def __init__(self):
        self.lastMessage = rospy.get_rostime()
        self.incomm = True
        self.simcomm = True


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


class DataListener:
    """ Listens to all of the applicable topics and repackages into a single object """

    def __init__(self, agent, agent_type):
        self.agent = agent  # Agent or Neighbor object
        self.odom_sub = \
            rospy.Subscriber('/' + self.agent.id + '/odometry',
                             Odometry, self.Receiver, 'odometry', queue_size=1)
        # self.goal_sub = \
        #     rospy.Subscriber('/' + self.agent.id + '/frontier_goal_pose',
        # TODO Need to check this is the right goal to compare
        self.goal_sub = \
            rospy.Subscriber('/' + self.agent.id + '/node_skeleton/next_turn_pose',
                             PoseStamped, self.Receiver, 'goal', queue_size=1)
        self.node_sub = \
            rospy.Subscriber('/' + self.agent.id + '/at_a_node',
                             Bool, self.Receiver, 'atnode', queue_size=1)
        self.beacon_sub = \
            rospy.Subscriber('/' + self.agent.id + '/beacons',
                             BeaconArray, self.Receiver, 'commBeacons', queue_size=1)
        self.artifact_sub = \
            rospy.Subscriber('/' + self.agent.id + '/artifact_array',
                             ArtifactArray, self.Receiver, 'newArtifacts', queue_size=1)

        if agent_type == 'base':
            self.map_sub = \
                rospy.Subscriber('/' + self.agent.id + '/occupied_cells_vis_array',
                                 MarkerArray, self.Receiver, 'map')
            # self.map_sub = \
            #     rospy.Subscriber('/' + self.agent.id + '/voxblox_node/occupied_nodes',
            #                      MarkerArray, self.MapReceiver, 'map')

    def Receiver(self, data, parameter):
        if self.agent.simcomm:
            setattr(self.agent, parameter, data)
            self.agent.lastMessage = rospy.get_rostime()

        # Throttle our updates to at least faster than the multi-agent node
        # Don't throttle an octomap or it doesn't update properly
        if parameter != 'map':
            rospy.sleep(0.5)


def getDist(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)


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
        yaw = yaw + getYaw(pose.orientation)

    pos.x = pos.x / float(len(history))
    pos.y = pos.y / float(len(history))
    pos.z = pos.z / float(len(history))
    yaw = yaw / float(len(history))

    return pos, yaw


class MultiAgent:
    """ Initialize a multi-agent node for the agent, publishes data for others and listens """

    def __init__(self):
        # Load parameters from the launch file
        self.id = rospy.get_param('multi_agent/vehicle', 'X1')
        self.type = rospy.get_param('multi_agent/type', 'robot')
        # Rate to run the node at
        self.rate = rospy.get_param('multi_agent/rate', 1)
        # Whether to use simulated comms or real comms
        self.useSimComms = rospy.get_param('multi_agent/simcomms', False)
        # Time without message for lost comm
        self.commThreshold = rospy.Duration(rospy.get_param('multi_agent/commThreshold', False), 2)
        # Distance to drop beacons automatically
        self.maxDist = rospy.get_param('multi_agent/dropDist', 50)
        # Minimum distance between junctions before dropping another beacon
        self.junctionDist = rospy.get_param('multi_agent/junctionDist', 10)
        # Whether to use turn detection to drop beacons
        self.turnDetect = rospy.get_param('multi_agent/turnDetect', True)
        # Number of beacons this robot is carrying
        self.numBeacons = rospy.get_param('multi_agent/numBeacons', 4)
        # Total number of potential beacons
        totalBeacons = rospy.get_param('multi_agent/totalBeacons', 8)
        # Potential robot neighbors to monitor
        neighbors = rospy.get_param('multi_agent/potentialNeighbors', 'X1,X2').split(',')
        baseTopic = rospy.get_param('multi_agent/baseTopic', '/Anchor/commcheck')

        # Static anchor position
        self.anchorPos = Point()
        self.anchorPos.x = rospy.get_param('multi_agent/anchorX', 1.0)
        self.anchorPos.y = rospy.get_param('multi_agent/anchorY', 0.0)
        self.anchorPos.z = rospy.get_param('multi_agent/anchorZ', 0.1)

        self.neighbors = {}
        self.beacons = {}
        self.comm_sub = {}
        self.simcomms = {}
        self.commcheck = {}
        self.artifacts = {}
        self.monitor = {}
        self.history = []
        self.hislen = self.rate * 10  # How long the odometry history should be
        self.report = False

        # Define which nodes republish information to display
        # Could add this to launch file, or make any 'base' type use it
        self.monitors = ['Anchor']

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
            DataListener(self.agent, 'robot')
            self.task_pub = rospy.Publisher('task', String, queue_size=10)
            self.deploy_pub = rospy.Publisher('deploy', Bool, queue_size=10)
            self.stop_pub = rospy.Publisher('e_stop', Bool, queue_size=10)

        # Initialize base station
        self.base = Base()
        self.base_sub = rospy.Subscriber(baseTopic, CommsCheckArray, self.BaseMonitor)

        if self.useSimComms:
            self.comm_sub[self.id] = \
                rospy.Subscriber('commcheck', CommsCheckArray, self.simCommChecker, self.id)
            self.comm_sub[self.id] = \
                rospy.Subscriber('/Anchor/commcheck', CommsCheckArray, self.simCommChecker, 'Anchor')

        # Get our 'number id' for beacon assignment
        if self.type == 'robot':
            sid = int(self.id[-1])
        else:
            sid = 0

        # Setup the beacons.  For real robots the names shouldn't matter as long as consistent
        for i in range(1, totalBeacons + 1):
            nid = 'B' + str(i)

            # Determine if this agent 'owns' the beacon so we don't have conflicting names
            owner = i <= sid * self.numBeacons and i > (sid - 1) * self.numBeacons

            self.beacons[nid] = BeaconObj(nid, owner)
            if self.useSimComms:
                comm_topic = '/' + nid + '/commcheck'
                self.comm_sub[nid] = \
                    rospy.Subscriber(comm_topic, CommsCheckArray, self.simCommChecker, nid)

        # Setup the listeners for each neighbor
        for nid in [n for n in neighbors if n != self.id]:
            self.neighbors[nid] = Neighbor(nid)
            DataListener(self.neighbors[nid], self.type)

            if self.useSimComms:
                comm_topic = '/' + nid + '/commcheck'
                self.comm_sub[nid] = \
                    rospy.Subscriber(comm_topic, CommsCheckArray, self.simCommChecker, nid)

            # Setup topics for visualization at whichever monitors are specified (usually base)
            if self.id in self.monitors:
                topic = 'neighbors/' + nid + '/'
                self.monitor[nid] = {}
                self.monitor[nid]['odometry'] = \
                    rospy.Publisher(topic + 'odometry', Odometry, queue_size=10)
                self.monitor[nid]['goal'] = \
                    rospy.Publisher(topic + 'goal', PoseStamped, queue_size=10)
                self.monitor[nid]['map'] = \
                    rospy.Publisher(topic + 'map', MarkerArray, queue_size=10)

        self.beacon_pub = rospy.Publisher('beacons', BeaconArray, queue_size=10)

        # Setup the beacon monitor
        if self.id in self.monitors:
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

            self.monitor['artifacts'] = rospy.Publisher('martifacts', Marker, queue_size=10)
            self.martifact = Marker()
            self.martifact.header.frame_id = 'world'
            self.martifact.type = self.martifact.CUBE_LIST
            self.martifact.action = self.martifact.ADD
            self.martifact.scale.x = 0.5
            self.martifact.scale.y = 0.5
            self.martifact.scale.z = 0.5
            self.martifact.color.a = 1.0
            self.martifact.color.r = 1.0
            self.martifact.color.g = 0.0
            self.martifact.color.b = 0.0

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

        self.martifact.points = []
        for artifact in self.artifacts.values():
            self.martifact.points.append(artifact.artifact.position)
        self.monitor['artifacts'].publish(self.martifact)

    def publishBeacons(self):
        commBeacons = []
        for beacon in self.beacons.values():
            commBeacon = Beacon()
            commBeacon.id = beacon.id
            commBeacon.active = beacon.active
            commBeacon.pos = beacon.pos
            commBeacons.append(commBeacon)

        self.beacon_pub.publish(commBeacons)

    def BaseMonitor(self, data):
        # For now just recording that we received a message
        if self.base.simcomm:
            self.base.lastMessage = rospy.get_rostime()

    def CommCheck(self):
        # Simply check when the last time we saw a message and set status
        for neighbor in self.neighbors.values():
            neighbor.incomm = neighbor.lastMessage > rospy.get_rostime() - self.commThreshold

        # Same with base station
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
            elif simcomm.id != 'Anchor':
                self.neighbors[simcomm.id].simcomm = simcomm.incomm

        # Base comms are whatever our status with the anchor is
        if self.simcomms and self.id != 'Anchor':
            self.base.simcomm = self.simcomms['Anchor'].incomm

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

    def deployBeacon(self, inplace):
        deploy = False
        # Find the first available beacon for this agent
        for beacon in self.beacons.values():
            if beacon.owner and not beacon.active:
                deploy = beacon.id
                break

        if deploy:
            # Stop the robot and publish message to deployment mechanism
            self.stop_pub.publish(True)
            self.deploy_pub.publish(True)

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

            print(self.id, "deploying beacon", deploy)
            try:
                if self.useSimComms:
                    # Drop the simulated beacon, and pause to simulate drop
                    ret = set_state(state)
                    rospy.sleep(3)
                    print(ret.status_message)
                else:
                    # Need to monitor a success message or something
                    # For now just sit to simulate a drop
                    rospy.sleep(10)

                # Resume the mission
                self.stop_pub.publish(False)
                self.deploy_pub.publish(False)

                self.numBeacons = self.numBeacons - 1
                self.beacons[deploy].active = True
                self.beacons[deploy].simcomm = True
                self.beacons[deploy].pos = pose.position
            except Exception as e:
                rospy.logerr('Error deploying beacon %s', str(e))
        else:
            print(self.id, "no beacon to deploy")

    def beaconCheck(self):
        # Check if we need to drop a beacon if we have any beacons to drop
        if self.numBeacons > 0:
            pose = self.agent.odometry.pose.pose
            numBeacons = 0
            numDistBeacons = 0
            dropBeacon = False

            # We're connected to the mesh, either through anchor or beacon(s)
            if self.base.incomm:
                # Update our movement history.  May need to move up one level if we use out of comm
                self.updateHistory()

                # Once we pass the maxDist we could set a flag so we don't keep recalculating this
                anchorDist = getDist(pose.position, self.anchorPos)
                # If we have different ranges for anchor-beacon and beacon-beacon, change this
                checkDist = self.maxDist

                # Distance based drop only kicks in once out of anchor range
                # If we're at end of anchor range, drop beacon
                if anchorDist > self.maxDist:
                    dropBeacon = True
                    checkDist = self.maxDist

                # Always drop a beacon if we're at a node and we're in comm
                # If beacons are strong enough may want to restrict distance
                if self.agent.atnode.data:
                    dropBeacon = True
                    checkDist = self.junctionDist
                # Check if it looks like we're going around a corner
                elif self.turnDetect and len(self.history) == self.hislen:
                    pos1, yaw1 = averagePose(self.history[:int(0.4 * self.hislen)])
                    pos2, yaw2 = averagePose(self.history[int(0.6 * self.hislen):])

                    # Check that we've turned and moved far enough, over history and last second
                    # Will need to retune these for real vehicle dynamics
                    if (getDist(pos1, pos2) > 4 and abs(yaw2 - yaw1) > 0.5 and
                            getDist(self.history[-2].position, self.history[-1].position) > 0.5):
                        dropBeacon = True
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

                # Cancel the drop if we have more than one beacon in range
                if numBeacons - numDistBeacons > 0:
                    dropBeacon = False

                if dropBeacon:
                    self.deployBeacon(True)
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
                        print(self.id, 'new artifact', artifact_id)

    def artifactCheck(self):
        # Check the artifact list received from the artifact manager for new artifacts
        for artifact in self.agent.newArtifacts.artifacts:
            artifact_id = (str(artifact.position.x) +
                           str(artifact.position.y) +
                           str(artifact.position.z))
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
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.useSimComms:
                self.simCommCheck()

            # Update incomm based on last message seen
            self.CommCheck()

            # Reconcile beacon list with neighbors', and publish the list
            self.updateBeacons()
            self.publishBeacons()

            # Update the visualization topics
            if self.id in self.monitors:
                self.baseArtifacts()
                self.publishNeighbors()

            # Non-robot nodes don't need to do the following
            if self.type == 'robot':
                # Check if we need to drop a beacon
                self.beaconCheck()
                # Make sure our internal artifact list is up to date, and if we need to report
                self.artifactCheck()

                # Change the goal to go home to report
                if self.report:
                    # If we're in comms, assume the current list is reported
                    # Eventually may want to add a confirmation from base
                    if self.base.incomm:
                        self.report = False
                        self.task_pub.publish('Explore')
                        for artifact in self.artifacts.values():
                            artifact.reported = True
                    else:
                        print(self.id, 'return to report...')
                        self.task_pub.publish('Home')

            rate.sleep()
        return


if __name__ == '__main__':
    ma = MultiAgent()

    try:
        ma.start()
    except rospy.ROSInterruptException:
        pass
