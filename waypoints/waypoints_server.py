#! /usr/bin/env python

"""
    Action server.

    Actionlib server that handles
    the navigation between the various
    waypoints.
"""

# Modules
import tf
import sys
import math
import rospy
import actionlib
import networkx as nx

# Topological map
from graphs import simulator_graph

# Routines
from helpers import GoToPose

# Messages
import geometry_msgs.msg
from std_msgs.msg import String
from move_base_msgs.msg import *

# Custom server action message
from human_aware_robot_navigation.msg import *

class WaypointServer:

    def __init__(self, name):
        """
            Constructor.

            Arguments:
                param1: server namespace
        """
        # Flag
        self.success = True

        # Detections message (subscription)
        self.detections = None

        # Movebase object
        self.gtp = GoToPose()

        # Frequency rate
        self.r = rospy.Rate(1)

        # Server namespace
        self.action_name = name

        # Transform object
        self.tf_listener = tf.TransformListener()

        # Subscriber to person detection
        self.detection_sub = rospy.Subscriber('person_detection', Detections, self.setDetections)

        # Action server object
        self.action_server = actionlib.SimpleActionServer(self.action_name,
                                                          WaypointsAction,
                                                          execute_cb = self.callback,
                                                          auto_start = False)

        # Spin server and set callback
        self.action_server.start()

    def callback(self, goal):
        """
            Action server callback
            function. It gets called
            every time the client sends
            a goal and pipes it through.

            Arguments:
                param1: node in the topological map
        """
        # Check if node is valid
        for node in simulator_graph.nodes():

            if goal.end_node == node[2]:
                # Successful node
                rospy.loginfo("Setting the following goal %s", goal)

                # Found closest node
                current_node = self.getClosestPoint()

                # Compute shortest path to goal
                path = nx.dijkstra_path(simulator_graph, current_node, node)

                # Move along computed path
                self.move_along_path(path)

                # Set goal status
                self.action_server.set_succeeded()

                # Break the logic
                return

        # Invalid node: Abort the action
        rospy.loginfo("Node %s not found in the map!", goal)
        self.action_server.set_aborted()

    def move_along_path(self, path):
        """
            Logic to move along the
            shortest computed path.

            Arguments:
                param1: djikstra path
        """
        # Path iterator
        i = 0

        # Send robot to closest
        # node until it reaches
        # the goal (iteratively)
        while i <= len(path)-1:
            # Send goal
            result = self.gtp.goto(path[i][0], path[i][1], 0)

            # If goal successful
            if result:
                # Succesfully reached state
                rospy.loginfo("Reached goal: %s", path[i][2])

                # Move to next goal
                i += 1
            else:
                # Log event
                rospy.loginfo("Something blocked the way to the goal...")

                # Check for human presence
                # in front of the robot
                if self.getDetections().number_of_detections > 0:
                    rospy.loginfo("Human presence detected: %s number of detections", self.getDetections().number_of_detections)

                    # Interact with the person
                    rospy.loginfo("Please move!")

                    # Try again the planning
                    continue

        # Successful movement (thank you djikstra)
        rospy.loginfo("Reached final goal: %s", path[i-1][2])

    def getClosestPoint(self):
        """
            Finds closest point
            in the topological map.

            Returns:
                node: closest node in the topological map
        """
        # Check if base_link transform
        # is valid
        if self.getPose():
            # Set closest node
            closest_node = None

            # Get robot's current position
            (pose, _) = self.getPose()
            print("Pose: ", pose)

            # Find minimum difference
            min_distance = sys.maxsize

            # Compute least squared
            for node in simulator_graph.nodes():
                x_diff = node[0] - pose[0]
                y_diff = node[1] - pose[1]
                distance = math.sqrt(x_diff**2 + y_diff**2)

                # Update distance if needed
                if distance < min_distance:
                    min_distance = distance
                    closest_node = node

            return closest_node

    def getPose(self):
        """
            Returns the base_link position
            in relation to the map.

            Returns:
                list: trans and quaternion of the ar marker (in the map)
        """
        try:
            return self.tf_listener.lookupTransform('/map', 'base_link', rospy.Time(0))
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo("getPose function lookup EXCEPTION: %s", e)

    def getDetections(self):
        """
            Getter method to
            retrieve setted
            detections messages.

            Returns:
                Detections: detections message
        """
        return self.detections

    def setDetections(self, data):
        """
            Setter method for
            detections message.

            Arguments:
                param1: Detections object
        """
        self.detections = data

def main(args):

    # Initialise node
    rospy.init_node('waypoints_server', anonymous=True)

    try:
        # Initialise waypoints node
        ws = WaypointServer("waypoints")

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)
