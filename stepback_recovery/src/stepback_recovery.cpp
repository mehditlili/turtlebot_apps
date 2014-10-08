/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
* Edited by: Mehdi Tlili
*********************************************************************/
#include <stepback_recovery/stepback_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(stepback_recovery, StepbackRecovery, stepback_recovery::StepbackRecovery, nav_core::RecoveryBehavior)

namespace stepback_recovery {
  StepbackRecovery::StepbackRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false), world_model_(NULL) {} 

void StepbackRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    //Stepping back 10 cm per default
    private_nh.param("stepback_length", stepback_length_, 0.1);
    private_nh.param("frequency", frequency_, 10.0);

    blp_nh.param("acc_lim_th", acc_lim_th_, 3.2);
    blp_nh.param("max_rotational_vel", max_rotational_vel_, 1.0);
    blp_nh.param("min_in_place_rotational_vel", min_rotational_vel_, 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

StepbackRecovery::~StepbackRecovery(){
  delete world_model_;
}


double StepbackRecovery::getTranslation(double x0, double y0){
	tf::StampedTransform transform;
	tf_->lookupTransform("/odom","/base_footprint",ros::Time(0), transform);
	geometry_msgs::TransformStamped msg;
	tf::transformStampedTFToMsg(transform, msg);
	double translation = (msg.transform.translation.x -x0)*(msg.transform.translation.x -x0) + (msg.transform.translation.y -y0)*(msg.transform.translation.y -y0);
	translation = sqrt(translation);
	return translation;
}

void StepbackRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the StepbackRecovery object cannot be NULL. Doing nothing.");
    return;
  }

  ROS_WARN("Stepback recovery behavior started.");

  //Get initial position
  tf::StampedTransform transform;
  tf_->lookupTransform("/odom","/base_footprint",ros::Time(0), transform);
  geometry_msgs::TransformStamped msg;
  tf::transformStampedTFToMsg(transform, msg);
  double x0 = msg.transform.translation.x;
  double y0 = msg.transform.translation.y;

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);


  bool stepped_back = false;


  while(n.ok()){

    //compute the distance left to move
    double dist_left = stepback_length_ - getTranslation(x0, y0);

    //compute the velocity that will let us stop by the time we reach the goal
    double vel = sqrt(acc_lim_th_ * dist_left);

    //make sure that this velocity falls within the specified limits
    vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = -vel;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0;

    vel_pub.publish(cmd_vel);

    //Check if already moved back the whole distance
    ROS_DEBUG("Stepback behavior: Distance left %f meters", dist_left);
    if(dist_left < 0.0)
      stepped_back = true;

    //if we're done with our stepping back... then return
    if(stepped_back)
      return;

    r.sleep();
  }
}
};
