#!/usr/bin/env python

# import roslib; roslib.load_manifest('rbx1_nav')
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import radians, pi
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan, CameraInfo
import numpy as np
# from matplotlib import pyplot as plt

class MoveBase():
    def img_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)[300:, :]
            color = (255,0,0)
            color_hsv = np.array(color,dtype="uint8")
            color_hsv = np.array([[color_hsv]],dtype="uint8")
            color_hsv = cv2.cvtColor(color_hsv,cv2.COLOR_BGR2HSV)
            color_hsv = color_hsv[0][0]

            lower = (color_hsv[0]-20 if color_hsv[0]-20 > 0 else 0,
                        30,
                        30)
            upper = (color_hsv[0]+20 if color_hsv[0]+20 < 180 else 255,
                        255,
                        255)

            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            mask = cv2.inRange(hsv_img, lower, upper)
            im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            sorted_by_perimiter = sorted(contours,key = lambda x: cv2.arcLength(x,False))

            if(len(sorted_by_perimiter)>0):
                selected_contour = sorted_by_perimiter[-1] #largest contour
                (x,y),radius = cv2.minEnclosingCircle(selected_contour)
                center = (int(x),int(y))
                radius = int(radius)
                # print("radius", radius)
                cv2.circle(hsv_img,center,radius,(0,255,0),2)

                # M = cv2.moments(selected_contour)
                # cX = int(M["m10"] / M["m00"])
                # cY = int(M["m01"] / M["m00"])

            cv2.imshow('title',hsv_img)
            cv2.waitKey(100)
            # print("image")
            #def find_angle(self, box_x, total_width, openning_angle):
            # print("center: ", center[0])
            # print("shape:" , hsv_img.shape[1])
            self.angle = int(np.rad2deg(self.find_angle(center[0], hsv_img.shape[1], self.OPENNING_ANGLE)))

            # print("angle", self.angle)
            if radius > 50:
                self.has_box = True
            else:
                self.has_box = False
            # if box_len > 100:
            #     center = self.box_xcenter(box)
            #     self.angle = self.find_angle(center, hsv_img.shape[1], self.OPENNING_ANGLE)
            #     print("center,self.angle" , center, self.angle)
            #     self.has_box = True
            # else:
            #     self.has_box = False


        except CvBridgeError as e:
            print(e)

    def scan_callback(self, data):
        if self.has_box:
            # angle_deg = np.rad2deg(self.angle)
            # print("angle:" , self.angle)
            range = (self.angle + 360) % 360

            while data.ranges[range] == 0:
                range = (range+1)%360


            print("range", range)
            print("distance:", data.ranges[range])



    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        self.bridge = CvBridge()
        self.OPENNING_ANGLE = np.deg2rad(60)
        self.has_box = False

        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        rospy.Subscriber('scan',LaserScan,self.scan_callback)


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

        if not rospy.is_shutdown():
            print("setting goal")
            self.goal = MoveBaseGoal()

            # x, y = input("give x,y")
            x = 2.38
            y = 0.42

            # Use the map frame to define goal poses
            self.goal.target_pose.header.frame_id = 'map'

            # Set the time stamp to "now"
            self.goal.target_pose.header.stamp = rospy.Time.now()

            self.goal.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))

            # Start the robot moving toward the goal
            self.move(self.goal)
        rospy.spin()

    def done_cb(self, terminal_state, result):
        print("done_cb")
        print(terminal_state, result)
    def active_cb(self):
        print("active_cb")
    def feedback_cb(self, feedback):
        print("feedback_cb")
        # print(feedback)

    def move(self, goal):
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal , self.done_cb, self.active_cb, self.feedback_cb)


        # Allow 1 minute to get there
        # finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))
        #
        # # If we don't get there in time, abort the goal
        # if not finished_within_time:
        #     self.move_base.cancel_goal()
        #     rospy.loginfo("Timed out achieving goal")
        # else:
        #     # We made it!
        #     state = self.move_base.get_state()
        #     if state == GoalStatus.SUCCEEDED:
        #         rospy.loginfo("Goal succeeded!")

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


    def identify_box(self, box_img):
      hsv = cv2.cvtColor(box_img, cv2.COLOR_RGB2HSV)
      hue = hsv[:, :, 0]
      sat = hsv[:, :, 1]
      return ((185 <= hue) & (hue <= 200) &
              ((100 <= sat) & (sat <= 120) |
               (65 <= sat) & (sat <= 75))).nonzero()

    def mark_box(self, ax, identified_box):
      ys, xs = identified_box
      ax.scatter(xs, ys, color='pink')

    def box_xcenter(self, identified_box):
      _, xs = identified_box
      return np.average(xs)

    def find_angle(self, box_x, total_width, openning_angle):
      half_width = 0.5 * total_width
      img_plane_z = half_width / np.tan(0.5 * openning_angle)
      x_from_center = box_x - half_width
      return np.arctan2(x_from_center, img_plane_z)

if __name__ == '__main__':
    try:
        MoveBase()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
