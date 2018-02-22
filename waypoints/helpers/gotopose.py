#!/usr/bin/env python

'''
    Copyright (c) 2015, Mark Silliman
    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# Modules
import rospy
import numpy as np
import actionlib

# Messages
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class GoToPose():

    def __init__(self):
        """
            Constructor.
        """
        # Boolean flag
        self.goal_sent = False

    	# Movebase action client
    	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Wait for server connection
        # and send custom goal for the
        # execution (allow 5s)
        if self.move_base.wait_for_server(rospy.Duration(5)):
            # Set shutdown callback
            rospy.on_shutdown(self.shutdown)
            rospy.loginfo("Movebase action server live!")
        else:
            rospy.loginfo("Action server connection FAILED...")

    def goto(self, x, y, theta):
        """
            Function to send goal
            to movebase action server.

            Arguments:
                param1: x pose in the map
                param2: y pose in the map
                param3: theta rotation
        """
        # Build desired pose
        pos = {'x': x, 'y' : y}
        quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
        rospy.loginfo("Go to (%s, %s) pose", pos['x'], pos['y'])

        # Set goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Send goal
        self.move_base.send_goal(goal)

    	# Wait for 60s for task completition
    	success = self.move_base.wait_for_result(rospy.Duration(60))

        # Get state of the action
        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False

        return result

    def shutdown(self):
        # Cancel goal if shutdown
        if self.goal_sent:
            self.move_base.cancel_goal()

        rospy.loginfo("Stopped move base execution due to shutdown...")
        rospy.sleep(1)
