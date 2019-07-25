#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy

from std_msgs.msg import Bool
from std_msgs.msg import String


class Neighbor(object):
    """ Data structure to hold pertinent information about other agents """

    def __init__(self, agent_id):
        self.id = agent_id
        self.lastCommCheckSend = rospy.get_rostime()
        self.lastCommCheckRecv = rospy.get_rostime()
        self.incomm = True


class CommsChecker:
    """ Initialize a comms check node for the agent """

    def __init__(self, agent):
        self.id = agent
        self.neighbors = {}
        self.comm_pub = {}
        self.send_pub = {}
        self.recv_sub = {}

        rospy.init_node(agent + '_comms_check')
        self.start_time = rospy.get_rostime()
        while self.start_time.secs == 0:
            self.start_time = rospy.get_rostime()

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

            send_topic = '/' + agent + '_control/' + neighbor + '/send'
            recv_topic = '/' + agent + '_control/' + neighbor + '/recv'
            comm_topic = '/' + agent + '/neighbors/' + neighbor + '/commcheck'

            print('Subscribing to [{}], publishing to [{}], [{}]'.format(recv_topic, send_topic, comm_topic))
            self.send_pub[neighbor] = rospy.Publisher('%s' % send_topic, String, queue_size=100)
            self.recv_sub[neighbor] = rospy.Subscriber('%s' % recv_topic, String, self.CommReceiver)
            self.comm_pub[neighbor] = rospy.Publisher(comm_topic, Bool, queue_size=100)

    def CommReceiver(self, data):
        readData = data.data.split('###')
        sequence = readData[2]
        nid = readData[3]
        message = readData[4]

        if sequence == 'CommCheck':
            self.neighbors[nid].incomm = True
            self.neighbors[nid].lastCommCheckRecv = rospy.get_rostime()
            if message == 'ReturnToSender':
                self.send_pub[nid].publish('###' + str(rospy.get_rostime()) + '###CommCheck###' + self.id + '###GoodComms')

    def start(self):
        rate = rospy.Rate(2)
        offset = rospy.Duration(5)
        while not rospy.is_shutdown():
            curtime = rospy.get_rostime()

            for neighbor in self.neighbors.values():
                self.send_pub[neighbor.id].publish('###' + str(curtime) + '###CommCheck###' + self.id + '###ReturnToSender')
                neighbor.lastCommCheckSend = curtime
                if (curtime > self.start_time + offset and
                        neighbor.lastCommCheckSend > curtime - offset and
                        neighbor.lastCommCheckRecv < curtime - offset):
                    neighbor.incomm = False
                    # print("lost comm with", neighbor.id, "from", self.id)

                self.comm_pub[neighbor.id].publish(neighbor.incomm)
            rate.sleep()
        return


if __name__ == '__main__':
    cc = CommsChecker(sys.argv[1])

    try:
        cc.start()
    except rospy.ROSInterruptException:
        pass
