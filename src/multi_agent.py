#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import copy
import pickle
import zlib
import rospy
import tf

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class Neighbor(object):
    """ Data structure to hold pertinent information about other agents """

    def __init__(self, agent_id):
        self.id = agent_id
        self.cid = String()
        self.odometry = Odometry()
        self.goal = PoseStamped()
        self.map = MarkerArray()
        self.lastMessage = rospy.get_rostime()
        self.incomm = True

        if 'B' in agent_id:
            self.active = False
        else:
            self.active = True

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
        # Low bandwith data strips the map, so don't overwrite
        if neighbor.map != '':
            self.map = neighbor.map

        # Update parameters depending on if we're talking directly or not
        if updater:
            self.active = True
            self.cid = updater
            self.lastMessage = rospy.get_rostime()
            self.incomm = True
        else:
            # Once an agent has been made active, they should always be active!
            if neighbor.active:
                self.active = True
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


class DataListener:
    """ Listens to all of the applicable topics and repackages into a single object """

    def __init__(self, agent):
        self.agent = agent
        self.odom_sub = rospy.Subscriber(self.agent.id + '/odometry', Odometry, self.Receiver, 'odometry')
        self.goal_sub = rospy.Subscriber(self.agent.id + '/frontier_goal_pose', PoseStamped, self.Receiver, 'goal')
        self.map_sub = rospy.Subscriber(self.agent.id + '/occupied_cells_vis_array', MarkerArray, self.Receiver, 'map')
        # self.map_sub = rospy.Subscriber(self.agent.id + '/voxblox_node/occupied_nodes', MarkerArray, self.MapReceiver, 'map')

    def Receiver(self, data, parameter):
        setattr(self.agent, parameter, data)


class MultiAgent:
    """ Initialize a multi-agent node for the agent, publishes data for others and listens """

    def __init__(self, agent):
        self.id = agent
        self.sendData = []
        self.neighbors = {}
        self.send_pub = {}
        self.recv_sub = {}
        self.monitor = {}
        self.recv_buffer = {}
        self.send_sequence = {}
        self.recv_sequence = {}
        self.payload_limit = 100000
        self.timer = 0

        # Define which nodes republish information to display
        self.monitors = ['Anchor']

        # Setup possible beacons to deploy for this agent
        if 'X' in agent:
            self.beacons = []
            self.beacons.append('B' + agent[1] + '1')
            self.beacons.append('B' + agent[1] + '2')

        rospy.init_node(agent + '_multi_agent')

        self.pubData = Agent(agent)

        # Setup the internal listeners for all data to be sent, but not beacons
        # TODO maybe not for anchor too?
        if 'B' not in agent:
            DataListener(self.pubData)

        # Get a list of the other agents from the comm control recv topics
        neighbors = []
        topics = rospy.get_published_topics('/' + agent + '_control')
        for topic in topics:
            line = topic[0].split('/')
            if line[3] == 'recv':
                neighbors.append(line[2])

        # Setup the publisher and subscriber to the comm topics for each pair
        for neighbor in neighbors:
            self.neighbors[neighbor] = Neighbor(neighbor)

            send_topic = '/' + agent + '_control/' + neighbor + "/send"
            recv_topic = '/' + agent + '_control/' + neighbor + "/recv"
            print('Subscribing to [{}], publishing to [{}]'.format(recv_topic, send_topic))
            self.send_pub[neighbor] = rospy.Publisher('%s' % send_topic, String, queue_size=100)
            self.recv_sub[neighbor] = rospy.Subscriber('%s' % recv_topic, String, self.CommReceiver)

            # Setup topics at the anchor for visualization of what it can see
            if agent in self.monitors and 'X' in neighbor:
                topic = '/' + agent + '/neighbors/' + neighbor + '/'
                self.monitor[neighbor] = {}
                self.monitor[neighbor]['odometry'] = rospy.Publisher(topic + 'odometry', Odometry, queue_size=10)
                self.monitor[neighbor]['goal'] = rospy.Publisher(topic + 'goal', PoseStamped, queue_size=10)
                self.monitor[neighbor]['map'] = rospy.Publisher(topic + 'map', MarkerArray, queue_size=10)

    def publishNeighbors(self):
        # Topics for visualization at the anchor node
        for neighbor in self.neighbors.values():
            if 'X' in neighbor.id:
                self.monitor[neighbor.id]['odometry'].publish(neighbor.odometry)
                self.monitor[neighbor.id]['goal'].publish(neighbor.goal)
                self.monitor[neighbor.id]['map'].publish(neighbor.map)

    def sendSequence(self, nid):
        if self.send_sequence[nid] < len(self.sendData):
            print('sending from', self.id, 'to', nid, self.send_sequence[nid] + 1, 'of', len(self.sendData))
            self.send_pub[nid].publish(self.sendData[self.send_sequence[nid]])

    def CommReceiver(self, data):
        # Load the current data from the individual neighbor
        readData = data.data.split('###')
        sequence = readData[1]
        nid = readData[2]
        message = readData[3]

        # Once we have the entire message, process the concatenated data
        if sequence == 'message_complete':
            if message != '':
                # print(self.id, 'received single message from', nid)
                neighbor = pickle.loads(zlib.decompress(message))
            else:
                print(self.id, 'received multipart message from', nid)
                self.send_pub[nid].publish('###Success###' + self.id + '###')
                neighbor = pickle.loads(zlib.decompress(self.recv_buffer[nid]))
                self.recv_buffer[nid] = ''
                self.recv_sequence[nid] = 0

                # If we're beacon receiving a message for the first time, activate,
                # and read the position data from our spawner
                if not self.pubData.active:
                    print("Activate", self.id, "!!!")
                    self.pubData.active = True
                    self.odometry = neighbor.neighbors[self.id].odometry

                self.updateData(neighbor)
        elif sequence == 'Success':
            self.send_sequence[nid] = self.send_sequence[nid] + 1
            self.sendSequence(nid)
        elif sequence == 'Retransmit':
            self.send_sequence[nid] = int(message)
            print("retransmitting", message, self.send_sequence[nid])
            self.sendSequence(nid)
        else:
            if sequence == '0':
                self.recv_buffer[nid] = message
                self.recv_sequence[nid] = 0
                self.send_pub[nid].publish('###Success###' + self.id + '###')
            else:
                nextSeq = self.recv_sequence[nid] + 1
                if sequence == str(nextSeq):
                    self.recv_buffer[nid] = self.recv_buffer[nid] + message
                    self.recv_sequence[nid] = nextSeq
                    self.send_pub[nid].publish('###Success###' + self.id + '###')
                else:
                    print("Missing", self.recv_sequence[nid], ", currently at", str(nextSeq))
                    self.recv_sequence[nid] = nextSeq
                    self.send_pub[nid].publish('###Retransmit###' + self.id + '###' + str(self.recv_sequence[nid]))

    def updateData(self, neighbor):
        # Update neighbor to the new information
        self.neighbors[neighbor.id].update(neighbor, self.id)

        # print(neighbor.id, self.neighbors[neighbor.id].cid, self.neighbors[neighbor.id].lastMessage.secs)
        # print(self.neighbors[neighbor.id].odometry)
        # print(self.neighbors[neighbor.id].goal)

        # Get our neighbor's neighbors data and update our own neighbor list
        for neighbor2 in neighbor.neighbors.values():
            # Make sure the neighbor isn't ourself, it's not a stale message,
            # and we're already talked directly to the neighbor in the last N seconds
            if (neighbor2.id != self.id and self.neighbors[neighbor2.id].lastMessage > rospy.Time(0) and
                    neighbor2.lastMessage >= self.neighbors[neighbor2.id].lastMessage and
                    (self.neighbors[neighbor2.id].cid != self.id or
                     self.neighbors[neighbor2.id].lastMessage <
                     rospy.get_rostime() - rospy.Duration(5))):
                self.neighbors[neighbor2.id].update(neighbor2)
                # print(neighbor2.id, self.neighbors[neighbor2.id].cid, self.neighbors[neighbor2.id].lastMessage.secs)
                # print(self.neighbors[neighbor2.id].goal)

        # Update the anchor's visualization topics
        if self.id in self.monitors:
            self.publishNeighbors()

    def spawnBeacon(self, inplace):
        spawn = False
        # Find the first available beacon for this agent
        for beacon in self.beacons:
            if not self.neighbors[beacon].active:
                spawn = beacon
                break

        if spawn:
            # For now we're just moving the beacon 3 units behind which will hopefull
            # be in range of the anchor or previous beacon
            # Either need to identify location before comm loss, or make this a guidance
            # command to return to a point in range
            service = '/gazebo/set_model_state'
            rospy.wait_for_service(service)
            set_state = rospy.ServiceProxy(service, SetModelState)

            pose = self.pubData.odometry.pose.pose
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
                self.neighbors[spawn].active = True
                self.neighbors[spawn].incomm = True
                self.neighbors[spawn].odometry.pose.pose = pose
                print(ret.status_message)
                self.runHighBandwidth()
            except Exception, e:
                rospy.logerr('Error calling service: modestate %s', str(e))
        else:
            print("no beacon to spawn")

    def start(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.pubData.active:
                self.pubData.neighbors = copy.deepcopy(self.neighbors)

                if self.timer % 5:
                    self.runLowBandwidth()
                elif self.id != 'Anchor':
                    self.runHighBandwidth()

                # Check if we need to drop a beacon if we're a vehicle
                if 'X' in self.id:
                    myPos = self.pubData.odometry.pose.pose.position
                    numBeacons = 0
                    dropBeacon = False
                    backBeacon = []
                    # Find how many beacons are active and that we can talk to
                    for neighbor in self.neighbors.values():
                        if 'B' in neighbor.id and neighbor.active:
                            backBeacon.append(False)
                            # Find the ones we're in comm with, and how far they are
                            if neighbor.incomm:
                                numBeacons = numBeacons + 1
                                pos = neighbor.odometry.pose.pose.position
                                dist = math.sqrt((myPos.x - pos.x)**2 + (myPos.y - pos.y)**2 + (myPos.z - pos.z)**2)

                                # TODO make this an adjustable parameter based on the comm system
                                if dist > 25:
                                    print(self.id, neighbor.id, dist, "beacon distance limit, let's drop!")
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

                        # TODO make this an adjustable parameter based on the comm system
                        if anchorDist > 25 and numBeacons == 0:
                            print("anchor distance limit, let's drop!")
                            dropBeacon = True

                    # If there are no beacons and we've lost comm we should try to drop
                    elif not self.neighbors['Anchor'].incomm and numBeacons == 0:
                        print("no anchor and no beacons, let's drop!")
                        backBeacon = [True]
                    elif numBeacons > 1:
                        dropBeacon = False

                    # Either drop a beacon in place or drop one behind if we lost comm
                    if dropBeacon:
                        self.spawnBeacon(True)
                    elif all(backBeacon):
                        self.spawnBeacon(False)

                self.timer = self.timer + 1

            rate.sleep()
        return

    def runLowBandwidth(self):
        # Strip the map data out for low bandwidth transmission
        pubData = copy.deepcopy(self.pubData)
        pubData.map = ''
        for neighbor in pubData.neighbors.values():
            neighbor.map = ''

        picklePubData = zlib.compress(pickle.dumps(pubData, 2), 9)
        # print(sys.getsizeof(picklePubData))
        # print(picklePubData)

        header = '###message_complete###' + self.id + '###'
        picklePubData = header + picklePubData

        for neighbor in self.neighbors.values():
            if neighbor.active:
                self.send_pub[neighbor.id].publish(picklePubData)

                # Check if we've received anything from this neighbor and update status
                if (neighbor.cid == self.id and
                        neighbor.lastMessage < rospy.get_rostime() - rospy.Duration(3)):
                    neighbor.incomm = False
                    print("lost comm with", neighbor.id, "from", self.id)

    def runHighBandwidth(self):
        rate = rospy.Rate(1)

        for neighbor in self.neighbors.values():
            # For now only sending map data to beacons and anchor
            # Therefore robots will not be able to relay map data
            if neighbor.active and 'X' not in neighbor.id:
                pubData = copy.deepcopy(self.pubData)
                picklePubData = zlib.compress(pickle.dumps(pubData, 2), 9)

                payload_size = sys.getsizeof(picklePubData)
                # print(payload_size)
                # print(picklePubData)

                sequence = 0
                start = 0
                stop = self.payload_limit

                while start < payload_size:
                    header = '###' + str(sequence) + '###' + self.id + '###'
                    self.sendData.append(header + picklePubData[start:stop])
                    start = stop
                    stop = start + self.payload_limit
                    sequence = sequence + 1

                self.sendData.append('###message_complete###' + self.id + '###')

                timeout = 0
                timer = 0
                # print('SEND from', self.id, 'to', neighbor.id, len(self.sendData))

                self.send_sequence[neighbor.id] = 0
                while self.send_sequence[neighbor.id] < len(self.sendData):
                    self.sendSequence(neighbor.id)

                    # If we haven't received a response in a timely manner, cancel
                    if timeout == self.send_sequence[neighbor.id]:
                        timer = timer + 1
                        # If we were already flagged out of comm, only try once
                        if timer > 5 or not neighbor.incomm:
                            break
                    else:
                        timeout = self.send_sequence[neighbor.id]
                        timer = 0

                    # Keep sending low bandwidth data while high is running
                    self.runLowBandwidth()
                    rate.sleep()

                self.sendData = []


if __name__ == '__main__':
    ma = MultiAgent(sys.argv[1])

    try:
        ma.start()
    except rospy.ROSInterruptException:
        pass
