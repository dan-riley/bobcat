#!/usr/bin/env python
from __future__ import print_function
import rospy

from marble_multi_agent.msg import MapDiffsResp
from marble_multi_agent.srv import GetMapDiffs, GetMapDiffsResponse
from octomap_merger.msg import OctomapArray
from octomap_merger.msg import OctomapNeighbors


class AgentDiffs(object):
    """ Helper object for storing map diffs from all agents """

    def __init__(self, agent_id):
        self.id = agent_id;
        self.mapDiffs = {}


class MapDiffs(object):
    """ Initialize map diff service to send missing map diffs to a client """

    def __init__(self):
        # Load parameters from the launch file
        self.id = rospy.get_param('multi_agent/vehicle', 'H01')
        mapDiffsTopic = rospy.get_param('multi_agent/mapDiffsTopic', 'map_diffs')
        self.neighbors = {}
        self.neighbors[self.id] = AgentDiffs(self.id)

        rospy.init_node(self.id + '_get_map_diffs')
        name = '/' + self.id + '/get_map_diffs'
        self.service = rospy.Service(name, GetMapDiffs, self.handle_get_map_diffs)

        topic = '/' + self.id + '/' + mapDiffsTopic
        self.map_sub = rospy.Subscriber(topic, OctomapArray, self.DiffReceiver, queue_size=10)

        # Could potentially get rid of this if we used a nodelet
        topic = '/' + self.id + '/neighbor_maps'
        self.map_sub = rospy.Subscriber(topic, OctomapNeighbors, self.NeighborDiffReceiver, queue_size=10)

        self.test_bw = False
        if self.test_bw:
            self.map_pub = rospy.Publisher('/' + self.id + '/bwtest', MapDiffsResp, queue_size=10)

        rospy.spin()

    def DiffReceiver(self, data):
        # Add any new diffs published by merger to our list
        for diff in data.octomaps:
            self.neighbors[self.id].mapDiffs[diff.header.seq] = diff

    def NeighborDiffReceiver(self, data):
        for nmap in data.neighbors:
            nid = nmap.owner
            # Make sure this neighbor is in our array
            if nid not in self.neighbors:
                self.neighbors[nid] = AgentDiffs(nid)

            # Add any new diffs saved by multi_agent to our list
            for diff in nmap.octomaps:
                self.neighbors[nid].mapDiffs[diff.header.seq] = diff

    def handle_get_map_diffs(self, req):
        resp = []
        for agent in req.agents:
            if agent.id in self.neighbors:
                nresp = MapDiffsResp()
                nresp.id = agent.id
                nresp.mapDiffs.owner = agent.id
                nresp.mapDiffs.num_octomaps = 0
                nresp.received = []

                # Add each requested diff to the message
                for i in agent.missing:
                    if i in self.neighbors[agent.id].mapDiffs:
                        nresp.mapDiffs.octomaps.append(self.neighbors[agent.id].mapDiffs[i])
                        nresp.mapDiffs.num_octomaps += 1
                        nresp.received.append(i)

                resp.append(nresp)

            if self.test_bw:
                self.map_pub.publish(nresp)

        # Publish the response
        return GetMapDiffsResponse(resp)


if __name__ == '__main__':
    md = MapDiffs()
