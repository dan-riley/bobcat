#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import graphviz

from PyQt5 import QtWidgets, QtSvg
from datetime import datetime
from bobcat.msg import BobcatStatus

class BOBCATViewer(object):
    """ Visualize bobcat_status topic data """

    def __init__(self):
        # Setup our variables
        self.id = rospy.get_param('bobcat_viewer/vehicle', 'H01')
        self.status = BobcatStatus()
        self.gv = graphviz.Digraph()
        self.colorscheme = '/blues9/'
        self.svg = None
        # Runtime flags
        self.showMonitorsToBehaviors = False  # Show the connections between monitors and behaviors
        self.trimMonitorsToBehaviors = True # Remove some of the connections that are extraneous
        self.saveGV = False  # Save the GV to a file
        self.saveSVG = False  # Save the SVG to a file
        # Initialize QT
        self.app = QtWidgets.QApplication(sys.argv)
        self.svgWidget = QtSvg.QSvgWidget()
        self.svgWidget.setWindowTitle('BOBCAT Status Viewer - ' + self.id)

        rospy.init_node('bobcat_viewer')
        self.status_sub = rospy.Subscriber('bobcat_status', BobcatStatus, self.StatusReceiver)

    def StatusReceiver(self, data):
        self.status = data
        # Convert the topic message to a graphviz object
        self.statusToGV()
        # Build an SVG image
        self.svg = bytearray(self.gv.pipe())
        # Display the image
        self.viewStatus()

    def statusToGV(self):
        g = graphviz.Digraph('bobcat_status', format='svg')
        g.attr(rankdir='LR', concentrate='true', splines='polyline', ranksep='1')
        g.attr('node', shape='box', fixedsize='true', width='2', height='1', style='filled,rounded')

        monitors = {}
        m = graphviz.Digraph('subgraph-m')
        for monitor in self.status.monitors:
            color = 'green' if monitor.status else '/blues9/2'
            # Only show red for comms, otherwise everything is red most of the time
            if monitor.name == 'Comms' and not monitor.status:
                color = 'red'
            m.node(monitor.name, fillcolor=color)
            monitors[monitor.name] = monitor.status

            # Create an intersection node to branch and use a single label
            status = 'T' if monitor.status else 'F'
            color = 'green' if monitor.status else 'red'
            hname = 'h' + monitor.name
            m.node(hname, shape='point', width='0')
            m.edge(monitor.name, hname, xlabel=status, color=color, dir='none', tailport='e')
        g.subgraph(m)

        objectives = {}
        o = graphviz.Digraph('subgraph-o')
        for objective in self.status.objectives:
            name = 'o' + objective.name
            color, ecolor, fontcolor = self.getColors(objective.weight)
            o.node(name, objective.name, fontcolor=fontcolor, fillcolor=color)
            objectives[objective.name] = objective.weight

            # Hidden node for branching
            hname = 'ho' + objective.name
            o.node(hname, shape='point', width='0')
            o.edge(name, hname, xlabel=str(round(objective.weight, 1)), color=ecolor, dir='none', tailport='e')

            # Objective to hidden monitor nodes
            for monitor in objective.monitors:
                color = 'green' if monitors[monitor] else 'red'
                o.edge('h' + monitor, name, color=color)

            # Hack to add a dummy monitor for Explore when it doesn't have one
            if objective.name == 'Explore' and not objective.monitors:
                m.node('ExploreToGoal', fillcolor='/blues9/2')
                m.node('hExploreToGoal', shape='point', width='0')
                o.edge('ExploreToGoal', 'hExploreToGoal', dir='none', xlabel='T', color='green')
                o.edge('hExploreToGoal', 'oExplore', color='green')
        g.subgraph(o)

        behaviors = {}
        b = graphviz.Digraph('subgraph-b')
        for behavior in self.status.behaviors:
            name = 'b' + behavior.name
            color, ecolor, fontcolor = self.getColors(behavior.score)
            b.node(name, behavior.name, fontcolor=fontcolor, fillcolor=color)

            # Behavior to hidden monitor nodes, if enabled
            if self.showMonitorsToBehaviors:
                for monitor in behavior.monitors:
                    if (not self.trimMonitorsToBehaviors or
                            (monitor != 'HumanInput' and monitor != 'NearbyRobot')):
                        color = 'green' if monitors[monitor] else 'red'
                        b.edge('h' + monitor, name, color=color)

            # Behavior to hidden objective nodes
            for objective in behavior.objectives:
                color, ecolor, fontcolor = self.getColors(objectives[objective])
                b.edge('ho' + objective, name, color=ecolor)
                if objective == 'Input':
                    b.edge('hoInput', 'bExplore', color=ecolor)
        g.subgraph(b)

        # Executed behavior and time
        e = graphviz.Digraph('subgraph-e')
        e.attr(rank='same')
        if self.status.execBehavior == 'Stop':
            color = 'red'
        elif self.status.execBehavior == 'GoHome':
            color = 'blue'
        elif self.status.execBehavior == 'DeployBeacon':
            color = 'orange'
        else:
            color = 'green'
        name = 'e' + self.status.execBehavior
        e.node(name, self.status.execBehavior, fillcolor=color, shape='circle', width='1.5')
        # Connect each behavior
        for behavior in self.status.behaviors:
            color, ecolor, fontcolor = self.getColors(behavior.score)
            g.edge('b' + behavior.name, name, xlabel=str(round(behavior.score, 1)), color=ecolor, tailport='e')

        # Build a time string from the timestamp
        timestr = datetime.utcfromtimestamp(self.status.header.stamp.to_sec()).strftime('%H:%M:%S')
        timestr = '<<font point-size="24.0">' + timestr + '</font>>'
        e.node('time', timestr, shape='plaintext', width='1.5', style='solid')
        g.subgraph(e)

        # Save the graph
        self.gv = g
        if self.saveGV:
            self.gv.save()
        if self.saveSVG:
            self.gv.render()

    def viewStatus(self):
        # Display our graph
        self.svgWidget.renderer().load(self.svg)
        self.svgWidget.show()

    def num2idx(self, n):
        # Convert a floating point number to a 1-9 index for colors
        nn = int(round(n * 2)) + 2
        if nn > 9:
            nn = 9
        return nn

    def getColors(self, n):
        # Get fill, edge and font colors for a particular index
        idx = self.num2idx(n)
        color = self.colorscheme + str(idx)
        if idx + 3 > 9:
            ecolor = 'black'
        else:
            ecolor = self.colorscheme + str(idx + 3)
        fontcolor = 'black'
        if idx == 9:
            fontcolor = 'white'

        return color, ecolor, fontcolor


if __name__ == '__main__':
    # Subscribe to status, build the message and display it
    viewer = BOBCATViewer()
    # Run until the window is closed
    sys.exit(viewer.app.exec_())
