#!/usr/bin/env python
from __future__ import print_function
import rospy

from marble_multi_agent.srv import GetMapDiffs, GetMapDiffsResponse
from marble_octomap_merger.msg import OctomapArray

class MapDiffs(object):
    """ Initialize map diff service to send missing map diffs to a client """

    def __init__(self):
        # Load parameters from the launch file
        self.id = rospy.get_param('multi_agent/vehicle', 'H01')
        mapDiffsTopic = rospy.get_param('multi_agent/mapDiffs', 'map_diffs')
        self.mapDiffs = {}

        rospy.init_node(self.id + '_get_map_diffs')
        name = '/' + self.id + '/get_map_diffs'
        self.service = rospy.Service(name, GetMapDiffs, self.handle_get_map_diffs)

        topic = '/' + self.id + '/' + mapDiffsTopic
        self.map_sub = rospy.Subscriber(topic, OctomapArray, self.DiffReceiver, queue_size=10)

        self.test_bw = False
        if self.test_bw:
            self.map_pub = rospy.Publisher('/' + self.id + '/bwtest', OctomapArray, queue_size=10)

        rospy.spin()

    def DiffReceiver(self, data):
        # Add any new diffs published by merger to our list
        for diff in data.octomaps:
            self.mapDiffs[diff.header.seq] = diff

    def handle_get_map_diffs(self, req):
        resp = OctomapArray()
        resp.owner = self.id
        resp.num_octomaps = 0
        received = []

        # Add each requested diff to the message
        for i in req.missing:
            resp.octomaps.append(self.mapDiffs[i])
            resp.num_octomaps += 1
            received.append(i)

        if self.test_bw:
            self.map_pub.publish(resp)

        # Publish the response
        return GetMapDiffsResponse(resp, received)


if __name__ == '__main__':
    md = MapDiffs()
