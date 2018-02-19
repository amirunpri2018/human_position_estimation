#! /usr/bin/env python

"""
    Action client.

    Creates an instance of
    the action client class
    and sends a custom goal to
    the action server for the
    execution.
"""

# Modules
from __future__ import print_function
import rospy
import actionlib

# Custom message
import actionlib_msgs.msg

class WaypointClient:

    def __init__(self):
        """ Constructor """
        # Action client instance
        self.action_client = actionlib.SimpleActionClient("waypoints", actionlib_msgs.msg.WaypointAction)

        # Wait for server connection
        # and send custom goal for the
        # execution
        if self.action_client.wait_for_server():
            self.setGoal()

    def setGoal(self):
        """
            Prepares custom goal
            and sends it to the
            action server for its
            execution.
        """
        # Create custom goal
        goal = actionlib_msgs.msg.WaypointGoal

        # Send goal to action server
        self.sendGoal(goal)

    def sendGoal(self, goal):
        """
            Receives custom goal
            and sends it to the
            action server.

            Arguments:
                param1: Custom actionlib goal
        """
        # Send goal to action server
        self.client.send_goal(goal)

        # Wait for server response
        self.client.wait_for_result()

        # Return result
        self.client.get_result()
