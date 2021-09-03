/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <actionlib/server/simple_action_server.h> 
#include <move_base_lite_msgs/FollowPathAction.h> 
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>


#include <memory>




using namespace teb_local_planner; // it is ok here to import everything for testing purposes
 ros::Publisher pub1;
class TestTebOptimNode
{
 public: 
    TestTebOptimNode() = default;
    tf2_ros::Buffer tfBuffer; 
    std::shared_ptr<actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction>> as_;
    void start(ros::NodeHandle& nh);
    
 protected:

   void followPathGoalCallback(); 
   void followPathPreemptCallback();
   actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction>::GoalConstPtr follow_path_goal_;
   move_base_lite_msgs::FollowPathResult result; 
   teb_local_planner::TebLocalPlannerROS planner; 
   costmap_2d::Costmap2DROS* costmap;
   bool planSet = false; 
};
 
//Function to start the local path planner
void TestTebOptimNode::start(ros::NodeHandle& nh)
{
    std::string map_frame = "world";
    tf2_ros::TransformListener tfListener(tfBuffer); 
    std::string name ="static_map"; 
    costmap_2d::Costmap2DROS costmap(name, tfBuffer);
    std::string name1 ="";
    planner.initialize(name1, &tfBuffer, &costmap); 
  
    //to get the global path
    as_.reset(new actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction>(nh, "/controller/follow_path", 0, false));
    as_->registerGoalCallback(boost::bind(&TestTebOptimNode::followPathGoalCallback, this));
    as_->registerPreemptCallback(boost::bind(&TestTebOptimNode::followPathPreemptCallback, this));
    as_->start();
    
    geometry_msgs::Twist vel1; 
    ros::Rate rate(20);
    while (ros::ok())
    {
     if(planSet)
        {
            //compute the velocity commands to follow the local path
            if (true == planner.computeVelocityCommands(vel1))  
            {
                pub1.publish(vel1);
            }
        }
     pub1 = nh.advertise<geometry_msgs::Twist> ("/cmd_vel_raw", 1);
       
       //to end the action
     if(planner.isGoalReached() && as_->isActive())
       {
           result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;  
           as_->setSucceeded(result,"reached goal");
       }       
        ros::spinOnce();
        rate.sleep();
    }
}

// =============== Main function =================
int main( int argc, char** argv )
{
   ros::init(argc, argv, "test_optim_node");
   ros::NodeHandle nh("~");
   TestTebOptimNode teb_test;
   teb_test.start(nh);

   return 0;
}


//function to get the global path
void TestTebOptimNode::followPathGoalCallback()
{
   follow_path_goal_ = as_->acceptNewGoal();
   std_msgs::Header header = follow_path_goal_->target_path.header;
   std::string id2= follow_path_goal_->target_path.header.frame_id;
   std::string id = follow_path_goal_->target_path.poses.at(0).header.frame_id;

   if( follow_path_goal_->target_path.poses.size() ==0)
   {
       result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
       as_->setSucceeded(result,"empty path");
       return;
   }

   std::vector<geometry_msgs::PoseStamped> final_path;
   final_path.push_back(follow_path_goal_->target_path.poses.at(0));
   final_path.at(0).header = header;
   //iterate over the global path
   for(unsigned int i=1; i< (follow_path_goal_->target_path.poses.size()-1); i++) 
   {
       geometry_msgs::PoseStamped p;
       p.pose = follow_path_goal_->target_path.poses.at(i).pose;
       p.header = header;
       float x = p.pose.position.x;
       float y = p.pose.position.y;
       if((x>= (2.5 + final_path.back().pose.position.x)) || (y>= (2.5 + final_path.back().pose.position.y)))  //to get all 2.5 meters a point
       {           
        final_path.push_back(p);
       }

   }
   final_path.push_back(follow_path_goal_->target_path.poses.back()); // save last entry as target point  
   final_path.back().header = header;
   nav_msgs::Path path2use = follow_path_goal_->target_path;
   path2use.poses.clear();
   for (size_t i = 0; i < final_path.size(); ++i) 
   {
        path2use.poses.push_back(final_path[i]);
   }

   //pass path to local path planner
   planner.setPlan(final_path);
   planner.customViaPointsCB(path2use);
   planSet = true;
}  

//to end the action
void TestTebOptimNode::followPathPreemptCallback()
{   
   geometry_msgs::Twist msg;
   msg.linear.x=0;
   msg.linear.y=0;
   msg.linear.z=0;
   msg.angular.x=0;
   msg.angular.y=0;
   msg.angular.z=0;
   pub1.publish(msg);
   ROS_INFO(" Preempted MPC Local Planner");
   as_->setPreempted();

}  
