#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

ros::Publisher goal_pub;
ros::Subscriber goal_sub;
geometry_msgs::PoseStamped goal_raw;
geometry_msgs::PoseStamped robot_pose;
costmap_2d::Costmap2D* costmap_;
bool initialized_ = false;
base_local_planner::WorldModel* world_model_;
costmap_2d::Costmap2DROS* costmap_ros_ = NULL;
costmap_2d::Costmap2DROS* global_costmap_ros_ = NULL;
double step_size_, min_dist_from_robot_;

void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, 0.08);
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.25);
      world_model_ = new base_local_planner::CostmapModel(*costmap_); 

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

double footprintCost(double x_i, double y_i, double theta_i)
{
  if(!initialized_){
    ROS_ERROR("The planner has not been initialized");
    return -1.0;
  }

  std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

  if(footprint.size() < 3)
    return -1.0;

  double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
  return footprint_cost;
}

bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
{
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized");
      return false;
    }
    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
    costmap_ = costmap_ros_->getCostmap();

/*    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }
*/
    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;

    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;
    double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

    double target_x = goal_x;
    double target_y = goal_y;
    double target_yaw = goal_yaw;

    bool done = false;
    double scale = 1.0;
    double dScale = 0.01;

    while(!done)
    {
      if(scale < 0)
      {
        target_x = start_x;
        target_y = start_y;
        target_yaw = start_yaw;
        ROS_WARN("The carrot planner could not find a valid plan for this goal");
        break;
      }
      target_x = start_x + scale * diff_x;
      target_y = start_y + scale * diff_y;
      target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);
      
      double footprint_cost = footprintCost(target_x, target_y, target_yaw);
      if(footprint_cost >= 0)
      {
          done = true;
      }
      scale -=dScale;
    }

    geometry_msgs::PoseStamped new_goal = goal;
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);
    new_goal.header.stamp = ros::Time::now();
    new_goal.header.frame_id = "map";

    new_goal.pose.position.x = target_x;
    new_goal.pose.position.y = target_y;

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

    goal_pub.publish(new_goal);
    return (done);
}

void CarrotGgoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& goal){

  goal_raw.pose.position.x = goal->pose.position.x;
  goal_raw.pose.position.y = goal->pose.position.y;
  goal_raw.pose.orientation = goal->pose.orientation;
  makePlan(robot_pose, goal_raw);
  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "carrot_goal");
  ros::NodeHandle n;
  tf::TransformListener tf(ros::Duration(5));

  goal_sub = n.subscribe<geometry_msgs::PoseStamped> ( "/move_base_simple/carrot_goal",2,CarrotGgoalCallBack);
  goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  geometry_msgs::TransformStamped transform_pose;
  
  ROS_INFO("Ready to Deal With MissionGoals.");

  ros::Rate rate(20.0);
  while (ros::ok())
  {

    try
	{
	  if(!initialized_)
	  {
            global_costmap_ros_ = new costmap_2d::Costmap2DROS("carrot_global_costmap", tf);
            global_costmap_ros_->pause();
            initialize("carrot_global_costmap",global_costmap_ros_);
	  }
	    listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
	    tf::transformStampedTFToMsg(transform,transform_pose);
	}
	catch (tf::TransformException ex)
	{
		ROS_DEBUG("%s",ex.what());
		continue;
	}

	robot_pose.pose.position.x = transform_pose.transform.translation.x;
	robot_pose.pose.position.y = transform_pose.transform.translation.y;
	robot_pose.pose.orientation = transform_pose.transform.rotation;
 	ros::spinOnce();
	rate.sleep();
  }

  return 0;
}

