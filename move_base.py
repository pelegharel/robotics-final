#!/usr/bin/env python

import roslib; roslib.load_manifest('rbx1_nav')
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import radians, pi

class MoveBase():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        while not rospy.is_shutdown():
            goal = MoveBaseGoal()

            x, y = input("give x,y")

            # Use the map frame to define goal poses
            goal.target_pose.header.frame_id = 'map'

            # Set the time stamp to "now"
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 0))

            # Start the robot moving toward the goal
            self.move(goal)

    def move(self, goal):
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)

        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))

        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBase()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
