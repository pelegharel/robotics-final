#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Wait ing for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS cost2d("mymap",tf);
  cost2d.start();

  auto* cost_map = cost2d.getCostmap();

  const cv::Mat mat_map(cost_map->getSizeInCellsY(),
                        cost_map->getSizeInCellsX(),
                        CV_8UC1,
                        cost_map->getCharMap());
  cv::imshow("mat_map", mat_map);

  tf::Stamped<tf::Pose> global_pose;
  cost2d.getRobotPose(global_pose);
  geometry_msgs::PoseStamped msg;
  tf::poseStampedTFToMsg(global_pose, msg);
  cout << msg << endl;
  cout << msg.pose.position.x << " " << msg.pose.position.y << endl;
  // cout << global_pose << endl;
  //std::cout >> global_pose;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 0;
  goal.target_pose.pose.position.x = 0;
  goal.target_pose.pose.orientation.w = 0.5;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
