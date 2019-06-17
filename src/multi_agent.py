#!/usr/bin/env python
import sys
import pickle
import rospy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray


class Neighbor(object):
    """ Data structure to hold pertinent information about other agents """

    def __init__(self, agent_id):
        self.id = agent_id
        self.cid = String
        self.odometry = Odometry
        self.goal = PoseStamped
        self.map = MarkerArray
        self.lastMessage = rospy.get_rostime()
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
        self.map = neighbor.map

        # Update parameters depending on if we're talking directly or not
        if updater:
            self.cid = updater
            self.lastMessage = rospy.get_rostime()
        else:
            self.cid = neighbor.cid
            self.lastMessage = neighbor.lastMessage

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
        # self.goal_sub = rospy.Subscriber(self.agent.id + '/frontier_goal_pose', PoseStamped, self.Receiver, 'goal')
        # self.map_sub = rospy.Subscriber(self.agent.id + '/voxblox_node/occupied_nodes', MarkerArray, self.MapReceiver, 'map')

    def Receiver(self, data, parameter):
        setattr(self.agent, parameter, data)


class MultiAgent:
    """ Initialize a multi-agent node for the agent, publishes data for others and listens """

    def __init__(self, agent):
        self.id = agent
        self.neighbors = {}
        self.send_pub = {}
        self.recv_sub = {}
        self.monitor = {}

        rospy.init_node(agent + '_multi_agent')

        self.pubData = Agent(agent)
        # Setup the internal listeners for all data to be sent
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
            if agent == 'Anchor':
                topic = '/Anchor/neighbors/' + neighbor + '/'
                self.monitor[neighbor] = {}
                self.monitor[neighbor]['odometry'] = rospy.Publisher(topic + 'odometry', Odometry, queue_size=10)
                self.monitor[neighbor]['goal'] = rospy.Publisher(topic + 'goal', PoseStamped, queue_size=10)

    def publishNeighbors(self):
        # Topics for visualization at the anchor node
        for neighbor in self.neighbors.values():
            self.monitor[neighbor.id]['odometry'].publish(neighbor.odometry)
            self.monitor[neighbor.id]['goal'].publish(neighbor.goal)

    def CommReceiver(self, data):
        # Load the current data from the individual neighbor
        neighbor = pickle.loads(data.data)

        # Update neighbor to the new information
        self.neighbors[neighbor.id].update(neighbor, self.id)

        print(neighbor.id, self.neighbors[neighbor.id].cid, self.neighbors[neighbor.id].lastMessage.secs)
        # print(self.neighbors[neighbor.id].odometry)
        print(self.neighbors[neighbor.id].goal)

        # Get our neighbor's neighbors data and update our own neighbor list
        for neighbor2 in neighbor.neighbors.values():
            # Make sure the neighbor isn't ourself, it's not a stale message,
            # and we're already talked directly to the neighbor in the last N seconds
            if (neighbor2.id != self.id and
                    neighbor2.lastMessage > self.neighbors[neighbor2.id].lastMessage and
                    (self.neighbors[neighbor2.id].cid != self.id or
                     self.neighbors[neighbor2.id].lastMessage <
                     rospy.get_rostime() - rospy.Duration(5))):
                self.neighbors[neighbor2.id].update(neighbor2)
                print(neighbor2.id, self.neighbors[neighbor2.id].cid, self.neighbors[neighbor2.id].lastMessage.secs)
                print(self.neighbors[neighbor2.id].goal)

        # Update the anchor's visualization topics
        if self.id == 'Anchor':
            self.publishNeighbors()

    def start(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Publish our data to each of the other agents
            self.pubData.neighbors = self.neighbors
            picklePubData = pickle.dumps(self.pubData)

            print(sys.getsizeof(picklePubData))
            # print(picklePubData)

            for neighbor in self.neighbors.values():
                self.send_pub[neighbor.id].publish(picklePubData)

            rate.sleep()
        return


if __name__ == '__main__':
    ma = MultiAgent(sys.argv[1])

    try:
        ma.start()
    except rospy.ROSInterruptException:
        pass
