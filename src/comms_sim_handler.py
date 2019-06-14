#!/usr/bin/env python
import sys
import threading
import rospy
from subt_example.srv import CreatePeer


class CommsHandler:
    """ Setup comms between two agents using the DARPA comms simulator """

    def __init__(self, source, dest):
        self.subData = None

        self.source = source
        self.dest = dest

        # Setup peer-connections topics from subt_example node
        service1 = '/' + source + '_control/create_peer'
        service2 = '/' + dest + '_control/create_peer'
        print("Waiting for services {} {}".format(service1, service2))

        rospy.wait_for_service(service1)
        rospy.wait_for_service(service2)
        print("Calling service {}".format(service1))
        rospy.ServiceProxy(service1, CreatePeer).call(dest)
        print("Calling service {}".format(service2))
        rospy.ServiceProxy(service2, CreatePeer).call(source)


class CommsRun(threading.Thread):
    def __init__(self, source, dest):
        self.comm = CommsHandler(source, dest)
        threading.Thread.__init__(self)


if __name__ == '__main__':
    # Run as: python2 comms_sim_handler.py X1 X2 X3 X4
    if len(sys.argv) < 3:
        print("Node requires at least 2 agent names")
        exit()

    # Start a connection between each agent in the list given
    i = 1
    for agent1 in sys.argv[i:]:
        for agent2 in sys.argv[i + 1:]:
            CommsRun(agent1, agent2)
        i += 1

    rospy.init_node('CommsHandler', anonymous=True)
