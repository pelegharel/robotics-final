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
from operator import add, sub
import tf
from tf.transformations import quaternion_from_euler
# from matplotlib import pyplot as plt


def dist_for_angle(ranges, angle):
    return ranges[(angle  + 360) % 360]

def calc_vec(angle, dist):
   angle = np.deg2rad(angle)
   return dist * np.array([np.sin(angle), np.cos(angle)])

def box_vector(ranges, theta_center):
    p_center = calv_vec(theta_center, dist_for_angle(angle))

def calc_goal(dist, angle):
    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(dist, 0, 0),
                                 Quaternion(*quaternion_from_euler(0,0,  np.deg2rad(angle))))
    return goal

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
        
            self.has_box = False 
            if(len(sorted_by_perimiter)>0):
                selected_contour = sorted_by_perimiter[-1] #largest contour
                (x,y),radius = cv2.minEnclosingCircle(selected_contour)
		x, y, w, h = cv2.boundingRect(selected_contour)
                cv2.rectangle(hsv_img,(x,y),(x+w, y+h),(0,255,0),2)

                self.angle = int(np.rad2deg(self.find_angle(x + (w/2), hsv_img.shape[1], self.OPENNING_ANGLE)))

                self.left_angle = int(np.rad2deg(self.find_angle(x, hsv_img.shape[1], self.OPENNING_ANGLE)))

	        self.right_angle = int(np.rad2deg(self.find_angle(x + (w), hsv_img.shape[1], self.OPENNING_ANGLE)))

                if w > 70 and (len(mask.nonzero()[1]) > 5000):
                    self.has_box = True
           
            cv2.imshow('title', cv2.cvtColor(hsv_img, cv2.COLOR_HSV2BGR))
            cv2.waitKey(100)

        except CvBridgeError as e:
            print(e)

    def print_dists(self, left_box_average,
                    left_out_box_average,
                    right_box_average,
                    right_out_box_average):
        print("--------------------------------------")
	print(left_out_box_average - left_box_average,
              right_out_box_average - right_box_average)

        print(left_out_box_average,
              left_box_average,
              right_box_average,
              right_out_box_average)


    def scan_callback(self, data):
        avg_calc_range = 10
        rotate_to_space_epsilon = 10
        if self.has_box:
            # angle_deg = np.rad2deg(self.angle)
            # print("angle:" , self.angle)
            angle_range = (self.angle + 360) % 360

            while data.ranges[angle_range] == 0:
                angle_range = (angle_range+1)%360
            
            (left_box_average,
             left_out_box_average,
             right_box_average,
             right_out_box_average) = [np.average([dist_for_angle(data.ranges, op(angle, i))
                                                   for i in range(avg_calc_range)])
                                       for angle, op in [(self.left_angle, sub),
                                                         (self.left_angle, add),
                                                         (self.right_angle, sub),
                                                         (self.right_angle, add)]]

            self.print_dists(left_box_average, left_out_box_average, right_box_average, right_out_box_average)

            if ((left_out_box_average- left_box_average) > 0.1):
                to_rotate = self.left_angle + rotate_to_space_epsilon
                print("will rotate:", to_rotate)
                self.move2(calc_goal(0, to_rotate))
           

    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        self.bridge = CvBridge()
        self.OPENNING_ANGLE = np.deg2rad(60)
        self.has_box = False
        self.moving = False
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

    def move2(self, goal):
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(30))

        if not finished:
             print("not finished within time")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                print("goal succeeded")

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
