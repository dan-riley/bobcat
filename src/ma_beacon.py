#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import Bool
from multi_agent import MultiAgent, BeaconObj


class MABeacon(MultiAgent):
    """ Initialize a multi-agent beacon node """

    def __init__(self):
        # Get all of the parent class variables
        MultiAgent.__init__(self)

        self.beacon = BeaconObj(self.id, self.id)
        self.raiseAntenna = rospy.Publisher('/' + self.id + '/set_mast', Bool, queue_size=10)

        self.commListen = True

    def beaconCommCheck(self, data):
        if not self.beacon.active:
            activate = False

            # Check the beacon data to see if we should be active yet
            # May not want to do this for real beacons, although could help with data rates
            for beacon in data.commBeacons.data:
                if beacon.id == self.id:
                    activate = True
                    self.beacon.active = True
                    self.beacon.pos = beacon.pos
                    self.raiseAntenna.publish(True)

            if not activate:
                return False

        return True

    def run(self):
        # Make sure our artifacts list is reconciled
        self.baseArtifacts()

        # If I'm a beacon, don't publish anything!  But keep listening for activate message.
        if not self.beacon.active:
            return False

        return True
