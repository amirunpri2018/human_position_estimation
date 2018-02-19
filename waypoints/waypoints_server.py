#! /usr/bin/env python

"""
    Action server.

    Actionlib server that handles
    the navigation between the various
    waypoints.
"""

# Modules
import rospy
import actionlib

# Custom message
import actionlib_msgs.msg

class WaypointServer:

    def __init__(self, name):
        """ Constructor. """

        # Frequency rate
        self.r = rospy.Rate(1)

        # Boolean flag
        self.success = True

        # Server namespace
        self.action_name = name

        # Action server object
        self.action_server = actionlib.SimpleActionServer(self.action_name, actionlib_msgs.msg.GoalStatus, execute_cb = self.execute_cb, auto_start = False)

        # Spin server
        self.action_server.start()

    def execute_cb(self, goal):
