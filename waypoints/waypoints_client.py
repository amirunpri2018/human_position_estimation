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
        """
            Constructor
        """
        # Action client instance
        self.action_client = actionlib.SimpleActionClient("waypoints", actionlib_msgs.msg.WaypointAction)

        # Wait for server connection
        # and send custom goal for the
        # execution (allow 5s)
        if self.action_client.wait_for_server(rospy.Duration(5)):
            self.setGoal()

    def setGoal(self, input_node):
        """
            Prepares custom goal
            and sends it to the
            action server for its
            execution.

            Arguments:
                param1: target node in the map
        """
        # Create custom goal
        goal = actionlib_msgs.msg.WaypointGoal
        goal.end_node = input_node

        # Send goal to action server
        self.client.send_goal(goal, self.doneCb, self.activeCb, self.feedbackCb)

    def doneCb(self, state, result):
        """
            Callback upon goal
            completition. Prints
            state and result of
            the goal.

            Arguments:
                param1: goal state
                param2: goal result
        """
        rospy.loginfo("Finished in state [%s]", state)
        rospy.loginfo("Goal result: %s", result)

    def activeCb(self):
        """
            Goal activation feedback.
        """
        rospy.loginfo("Goal went active!")

    def feedbackCb(self, feedback):
        """
            Feedback callback.

            Arguments:
                param1: goal feedback
        """
        rospy.loginfo("Received feedback: %s", feedback)
