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
        self.showMonitorsToBehaviors = True  # Show the connections between monitors and behaviors
        self.trimMonitorsToBehaviors = False # Remove some of the connections that are extraneous
        self.splitInputs = True # Split the Human Input Monitors into separate ones
        # Files save to ~/.ros if run via roslaunch.  SVG and EPS also save GV file.
        self.saveGV = False  # Save the GV to a file
        self.saveSVG = False  # Save the SVG to a file
        self.saveEPS = False  # Save the EPS to a file
        # When exporting make all monitor false's red; otherwise more neutral for attention
        self.falseColor = '/blues9/2'
        if self.saveSVG or self.saveEPS:
            self.falseColor = 'red'

        # Initialize QT
        self.app = QtWidgets.QApplication(sys.argv)
        self.svgWidget = QtSvg.QSvgWidget()
        self.svgWidget.setWindowTitle('BOBCAT Status Viewer - ' + self.id)

        rospy.init_node('bobcat_viewer')
        self.status_sub = rospy.Subscriber('bobcat_status', BobcatStatus, self.StatusReceiver)

        self.inputCommands = ['GoHome', 'DeployBeacon', 'Stop', 'GoToGoal', 'Explore']

    def StatusReceiver(self, data):
        self.status = self.correctInput(data)
        # Convert the topic message to a graphviz object
        self.statusToGV()
        # Build an SVG image
        self.svg = bytearray(self.gv.pipe())
        # Display the image
        self.viewStatus()

    def statusToGV(self):
        g = graphviz.Digraph('bobcat_status', format='svg')
        g.attr(rankdir='LR', concentrate='true', splines='polyline', ranksep='0.75', nodesep='0.13')
        g.attr('node', shape='box', fixedsize='true', width='2', height='1', style='filled,rounded')

        monitors = {}
        m = graphviz.Digraph('subgraph-m')
        # Keeps all of the input monitors together
        m.attr('node', ordering='out')
        for monitor in self.status.monitors:
            if self.splitInputs and monitor.name == 'HumanInput':
                for inputCommand in self.inputCommands:
                    color = 'green' if monitor.status and self.inputCompare(inputCommand) else self.falseColor
                    m.node('i' + inputCommand, fillcolor=color, fontsize='20', height='0.30')
            else:
                color = 'green' if monitor.status else self.falseColor
                height = '0.3' if 'i' in monitor.name[0] else '1'
                # Only show red for comms usually, otherwise everything is red most of the time
                if monitor.name == 'Comms' and not monitor.status:
                    color = 'red'
                m.node(monitor.name, fillcolor=color, fontsize='20', height=height)
            monitors[monitor.name] = monitor.status

            # Create an intersection node to branch and use a single label
            hname = 'h' + monitor.name
            m.node(hname, shape='point', width='0')
            if self.splitInputs and monitor.name == 'HumanInput':
                for inputCommand in self.inputCommands:
                    status = '1' if monitor.status and self.inputCompare(inputCommand) else '0'
                    color = 'green' if monitor.status and self.inputCompare(inputCommand) else 'red'
                    m.edge('i' + inputCommand, hname, xlabel=status, color=color, dir='none', tailport='e')
            else:
                status = '1' if monitor.status else '0'
                color = 'green' if monitor.status else 'red'
                penwidth = '4' if monitor.name == 'HumanInput' else '1'
                m.edge(monitor.name, hname, xlabel=status, color=color, dir='none', tailport='e', penwidth=penwidth)
        g.subgraph(m)

        objectives = {}
        o = graphviz.Digraph('subgraph-o')
        for objective in self.status.objectives:
            name = 'o' + objective.name
            color, ecolor, fontcolor = self.getColors(objective.weight)
            o.node(name, objective.name, fontcolor=fontcolor, fillcolor=color, fontsize='20')
            objectives[objective.name] = objective.weight

            # Hidden node for branching
            hname = 'ho' + objective.name
            o.node(hname, shape='point', width='0')
            o.edge(name, hname, xlabel=str(round(objective.weight, 1)), color=ecolor, dir='none', tailport='e')

            # Objective to hidden monitor nodes
            for monitor in objective.monitors:
                style = ''
                color = 'green' if monitors[monitor] else 'red'
                # Hide the connection between FindArtifacts and ExploreToGoal
                if objective.name == 'FindArtifacts' and monitor == 'ExploreToGoal':
                    style = 'invis'
                o.edge('h' + monitor, name, color=color, style=style)
        g.subgraph(o)

        behaviors = {}
        b = graphviz.Digraph('subgraph-b')
        for behavior in self.status.behaviors:
            name = 'b' + behavior.name
            color, ecolor, fontcolor = self.getColors(behavior.score)
            b.node(name, behavior.name, fontcolor=fontcolor, fillcolor=color, fontsize='20')

            # Behavior to hidden monitor nodes, if enabled
            if self.showMonitorsToBehaviors:
                for monitor in behavior.monitors:
                    if not self.trimMonitorsToBehaviors or monitor != 'HumanInput':
                        ename = 'h' + monitor
                        headport = '_'
                        if monitor == 'HumanInput':
                            ename = 'i' + behavior.name if self.splitInputs else ename
                            color = 'green' if monitors[monitor] and self.inputCompare(behavior.name) else 'red'
                        else:
                            color = 'green' if monitors[monitor] else 'red'
                        if self.splitInputs and monitor == 'ExploreToGoal' and behavior.name == 'GoToGoal':
                            headport = 'nw'
                        b.edge(ename, name, color=color, style='dashed', headport=headport)

            # Behavior to hidden objective nodes
            for objective in behavior.objectives:
                color, ecolor, fontcolor = self.getColors(objectives[objective])
                b.edge('ho' + objective, name, color=ecolor)
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
        e.node(name, self.status.execBehavior, fillcolor=color, shape='circle', width='1.75', fontsize='20')
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
        timestr = datetime.utcfromtimestamp(self.status.header.stamp.to_sec()).strftime('%H_%M_%S')
        filename = self.gv.name + '_' + self.id + '_' + timestr + '.gv'
        if self.saveGV:
            self.gv.save()
        if self.saveSVG:
            self.gv.render(filename=filename)
        if self.saveEPS:
            self.gv.render(filename=filename, format='eps')

    def viewStatus(self):
        # Display our graph
        self.svgWidget.renderer().load(self.svg)
        self.svgWidget.show()

    def inputCompare(self, compare):
        return self.status.inputCommand == compare

    def correctInput(self, status):
        # Correct input commands so monitor connections are cleaner
        if status.inputCommand == 'None':
            status.inputCommand = 'Explore'
        elif status.inputCommand == 'home':
            status.inputCommand = 'GoHome'
        else:
            status.inputCommand = status.inputCommand[0].upper() + status.inputCommand[1:]

        monitors = []
        if self.splitInputs:
            for monitor in status.monitors:
                if monitor.name == 'HumanInput':
                    for inputCommand in self.inputCommands:
                        newmonitor = lambda: None
                        newmonitor.name = 'i' + inputCommand
                        newmonitor.status = True if monitor.status and status.inputCommand == inputCommand else False
                        monitors.append(newmonitor)
                else:
                    monitors.append(monitor)
            status.monitors = monitors

        for objective in status.objectives:
            if self.splitInputs:
                monitors = []
                for monitor in objective.monitors:
                    if monitor == 'HumanInput':
                        for inputCommand in self.inputCommands:
                            monitors.append('i' + inputCommand)
                    else:
                        monitors.append(monitor)
                objective.monitors = monitors

            if objective.name == 'Explore':
                objective.name = 'FindArtifacts'

                # Add a dummy monitor for Explore when it doesn't have one
                if not objective.monitors:
                    newmonitor = lambda: None
                    newmonitor.name = 'ExploreToGoal'
                    newmonitor.status = False
                    status.monitors.append(newmonitor)
                    objective.monitors.append(monitor.name)

        for behavior in status.behaviors:
            if self.splitInputs:
                monitors = []
                for monitor in behavior.monitors:
                    if monitor == 'HumanInput':
                        monitors.append('i' + behavior.name)
                    else:
                        monitors.append(monitor)
                behavior.monitors = monitors

            objectives = []
            for objective in behavior.objectives:
                if objective == 'Explore':
                    objective = 'FindArtifacts'
                objectives.append(objective)

            # Old bags where Input not connected to Explore
            if behavior.name == 'Explore' and 'Input' not in behavior.objectives:
                objectives.append('Input')

            behavior.objectives = objectives

        return status

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
