#!/usr/bin/env python

# ros libraries
import rospy
import actionlib
from std_msgs.msg import String
import tf2_ros
import geometry_msgs.msg


#move_base_msgs
from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatus

# graph libraries
import networkx as nx
from upstairs import graph

# the graph
from move_along_roads.msg import *

import math

""" this is the big behind the scenes program for the main road system,
    modify or duplicate this to change how it works """

class RoadPlanner(object):
    """This class is used to calculate and execute the shortest path on the road
       network to get an agent to move there based on a graph that must be supplied."""
    # create messages that are used to publish feedback/result
    _feedback = move_along_roads.msg.RoadPlannerFeedback()
    _result   = move_along_roads.msg.RoadPlannerResult()

    # pretty self explanatory
    def move_to_node(self, node):

        move_goal = MoveBaseGoal ()
        move_goal.target_pose.header.frame_id = "map"

        # set the co-ordinates to move to
        x_goal = node[0]
        y_goal = node[1]

        move_goal.target_pose.pose.position.x = x_goal
        move_goal.target_pose.pose.position.y = y_goal
        # this seriously needs fixing - who wants the robot to point in weird
        # directions halfway along a corridor
        move_goal.target_pose.pose.orientation.w = 1.0

        ac.send_goal(move_goal)
        print "Attempting to move to node: " + node[2]
        return

    # constantly calculates the closest node and moves to it
    def get_onto_road(self):


        # listener.waitForTransform('map', 'base_link', rospy.Time(), rospy.Duration())

        # try:
        trans = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     print "\n\n\nThere was an exception\n\n"
        #     return

        robot_position = trans.transform.translation

        closest_node = None
        print "Position: (" + str(robot_position.x) + ", " + str(robot_position.y) + ")"
        while True:
            # reset shortest distance
            shortest_distance = 999999
            # get current position
            # compare to all the nodes, and come up with the closest
            for node in graph.nodes():
                x_diff = node[0] - robot_position.x
                y_diff = node[1] - robot_position.y
                distance = math.sqrt (x_diff**2 + y_diff**2)
                # if the distance is the shortest so far update the shortest distance
                if distance < shortest_distance:
                    shortest_distance = distance
                    closest_node_so_far = node

            # if there's a closer node than the one we're on then get on it
            if closest_node_so_far != closest_node:
                closest_node = closest_node_so_far
                """ this next line is completely optional and can be commented out"""
                # self.move_to_node (closest_node)
                return closest_node

            # if we're on a node then we're on the road B-)
            # if shortest_distance <= 0.5:
            #     print closest_node
            #     return closest_node

    # this is executed every time the road_planner gets a goal
    # it sets off the chain of commands that (hopefully) make the robot get to
    # where it's supposed to go
    def goalCB(self):
        goal_ =  self.as_.accept_new_goal().end_node
        print ("Target: " + goal_ + "\n")

        #look for the node in the graph
        is_a_node = False
        for node in graph.nodes():
            if node[2] == goal_:
                target_node = node
                is_a_node = True
                print ("Node found in graph: " + node[2] + "\n")

        if (is_a_node == False):
            print ("Node not found in graph: " + goal_ + "\n")
            self.as_.set_aborted()
            return

        # return the closest node
        current_node = self.get_onto_road()

        print "Closest node found:"
        print current_node
        print "\n"

        path = nx.dijkstra_path (graph, current_node, target_node)

        print "Shortest path computed:"
        print path
        print "\n"

        i = 1
        while i <= len(path):
            # can't really find a do...while in python
            self.move_to_node(path[-i])
            ac.wait_for_result()
            print "Waiting for result...\n"
            # print ac.get_result()

            if ac.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Reached node: " + path[-i][2] + "\n")
                # if the node we reached is the last node break
                if i == 1:
                    break
                # if not reset i and start all over again
                # NB the robot shouldn't go further back than the last node it reached
                else:
                    i = 1
                    continue
            else:
                rospy.loginfo("Couldn't get to the target node, trying next closest node...\n")
                i+=1
        # if we couldn't even reach the first node
        if i >= len(path):
            rospy.loginfo("Ran out of nodes, couldn't get to any of them")
            self.as_.set_aborted()


        #""" THIS IS TO BE USED IN CASE WE DON'T WANT TO PLAN TO THE END STRAIGHT AWAY """
        # for node in path:
        #     i = 1
        #     # can't really find a do...while in python
        #     while True:
        #         self.move_to_node(node)
        #         ac.wait_for_result()
        #         print "Waiting for result..."
        #         print ac.get_result()
        #
        #         if ac.get_state() == GoalStatus.SUCCEEDED:
        #             print "Reached node..."
        #             break
        #         else:
        #             print "Couldn't get to the node, trying again..."
        #         i+=1
        #         print i

        else:
            self.as_.set_succeeded()

        print ("Awaiting new goal")
        return


    # initialise the node
    def __init__(self, name):

        self.action_name_ = name
        self.as_ = actionlib.SimpleActionServer(self.action_name_, move_along_roads.msg.RoadPlannerAction, auto_start = False)
        self.as_.start()
        self.as_.register_goal_callback(self.goalCB)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)




if __name__ == '__main__':
    rospy.init_node('road_planner')

    print ("Initialising action client...")
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

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
