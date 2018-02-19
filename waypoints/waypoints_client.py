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

def main():
    """
        Main.
    """
    # Initialise node
    rospy.loginfo("Initialising node...")
    rospy.init_node('waypoints_cli')

    # Initialise action client
    print ("Initialising action client...")
    wc = WaypointClient()

    print ("Waiting for the move_base action server to come up...")
    ac.wait_for_server(rospy.Duration())
    print ("move_base server up")

    current_position = MoveBaseGoal()

    print ("Subscribing to robot_pose")
    rospy.Subscriber("robot_pose", MoveBaseGoal)

    print ("Initialising action server")
    server = RoadPlanner('road_planner')
    print ("Server up: Enter first destination node")

    rospy.spin()

if __name__ == '__main__':
    main()
