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
    tf2_ros::Buffer tfBuffer; //NB
    std::shared_ptr<actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction>> as_; //NB
    void start(ros::NodeHandle& nh);
    

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
protected:
//PlannerInterfacePtr planner;
/*TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;
boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;
ros::Subscriber custom_obst_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;
std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;

// =========== Function declarations =============
void CB_mainCycle(const ros::TimerEvent& e);
void CB_publishCycle(const ros::TimerEvent& e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);
void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);
void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id);*/
 void followPathGoalCallback(); //NB1
 void followPathPreemptCallback();
 actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction>::GoalConstPtr follow_path_goal_;
 move_base_lite_msgs::FollowPathResult result; //NB
 teb_local_planner::TebLocalPlannerROS planner; //NB
 costmap_2d::Costmap2DROS* costmap;
 bool planSet = false; 
 };
 
 void TestTebOptimNode::start(ros::NodeHandle& nh)
 {
 
    std::string map_frame = "world";
    
     
    tf2_ros::TransformListener tfListener(tfBuffer); //
    std::string name ="static_map"; //NB
    costmap_2d::Costmap2DROS costmap(name, tfBuffer); //NB   
    ROS_INFO("Hallo");
    //ros::Subscriber occ_map = nh.subscribe("traversability_occ_map",1, &TestMpcOptimNode::costmap_update, this);
    
    std::string name1 ="";
    planner.initialize(name1, &tfBuffer, &costmap); //NB 
    
    ROS_INFO("JA"); 
    
    as_.reset(new actionlib::SimpleActionServer<move_base_lite_msgs::FollowPathAction>(nh, "/controller/follow_path", 0, false));
    as_->registerGoalCallback(boost::bind(&TestTebOptimNode::followPathGoalCallback, this));
    as_->registerPreemptCallback(boost::bind(&TestTebOptimNode::followPathPreemptCallback, this));
    as_->start();
    
    geometry_msgs::Twist vel1; //NB
    ros::Rate rate(20);
    while (ros::ok())
    {

    
     if(planSet)
        {

            if (true == planner.computeVelocityCommands(vel1))  // NB
            {
                ROS_INFO_STREAM("CMDVEL="<<vel1.linear.x);
                pub1.publish(vel1);
            }
        }
        
     pub1 = nh.advertise<geometry_msgs::Twist> ("/cmd_vel_raw", 1);//NB
        
     if(planner.isGoalReached() && as_->isActive())
       {
           result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;  
           as_->setSucceeded(result,"reached goal");
       }
       
       
        ros::spinOnce();
        rate.sleep();
    }








 /*
  
  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);
 
  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.025), CB_mainCycle);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);
  
  // setup dynamic reconfigure
  dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(n);
  dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
  dynamic_recfg->setCallback(cb);
  
  // setup callback for custom obstacles
  custom_obst_sub = n.subscribe("obstacles", 1, CB_customObstacle);
  
  // setup callback for clicked points (in rviz) that are considered as via-points
  clicked_points_sub = n.subscribe("/clicked_point", 5, CB_clicked_points);
  
  // setup callback for via-points (callback overwrites previously set via-points)
  via_points_sub = n.subscribe("via_points", 1, CB_via_points);

  // interactive marker server for simulated dynamic obstacles
  interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");

  obst_vector.push_back( boost::make_shared<PointObstacle>(-3,1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(6,2) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,1.5,1,-1.5) ); //90 deg
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,0,-1,0) ); //180 deg
//  obst_vector.push_back( boost::make_shared<PointObstacle>(-1.5,-0.5) );

  // Dynamic obstacles
  Eigen::Vector2d vel (0.1, -0.3);
  obst_vector.at(0)->setCentroidVelocity(vel);
  vel = Eigen::Vector2d(-0.3, -0.2);
  obst_vector.at(1)->setCentroidVelocity(vel);

  /*
  PolygonObstacle* polyobst = new PolygonObstacle;
  polyobst->pushBackVertex(1, -1);
  polyobst->pushBackVertex(0, 1);
  polyobst->pushBackVertex(1, 1);
  polyobst->pushBackVertex(2, 1);
 
  polyobst->finalizePolygon();
  obst_vector.emplace_back(polyobst);
  */
  
 /* for (unsigned int i=0; i<obst_vector.size(); ++i)
  {
    // setup callbacks for setting obstacle velocities
    std::string topic = "/test_optim_node/obstacle_" + std::to_string(i) + "/cmd_vel";
    obst_vel_subs.push_back(n.subscribe<geometry_msgs::Twist>(topic, 1, boost::bind(&CB_setObstacleVelocity, _1, i)));

    //CreateInteractiveMarker(obst_vector.at(i)[0],obst_vector.at(i)[1],i,&marker_server, &CB_obstacle_marker);  
    // Add interactive markers for all point obstacles
    boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(obst_vector.at(i));
    if (pobst)
    {
      CreateInteractiveMarker(pobst->x(),pobst->y(),i, config.map_frame, &marker_server, &CB_obstacle_marker);  
    }
  }
  marker_server.applyChanges();
  
  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(n, config));
  
  // Setup robot shape model
  RobotFootprintModelPtr robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n);
  
  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
    planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
  else
    planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));
  

  no_fixed_obstacles = obst_vector.size();*/
  
  
  

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


void costmap_update(const nav_msgs::OccupancyGridConstPtr& new_map) 
{
   // costmap->incomingMap(new_map);
}
	
void TestTebOptimNode::followPathGoalCallback()
{
    follow_path_goal_ = as_->acceptNewGoal();
    //std::vector<geometry_msgs::PoseStamped> path = follow_path_goal_->target_path.poses;
    std_msgs::Header header = follow_path_goal_->target_path.header;
    std::string id2= follow_path_goal_->target_path.header.frame_id;
    ROS_INFO_STREAM("TOPLEVELHEADER="<< id2);
    ROS_INFO_STREAM("TOPLEVELSTAMP"<< follow_path_goal_->target_path.header.stamp);
    
    std::string id = follow_path_goal_->target_path.poses.at(0).header.frame_id;
    ROS_INFO_STREAM("ERSTER EINTRAG= "<< id);
    ROS_INFO_STREAM("ERSTER STAMP=" <<follow_path_goal_->target_path.poses.at(0).header.stamp);

    if( follow_path_goal_->target_path.poses.size() ==0)
    {
        result.result.val = move_base_lite_msgs::ErrorCodes::SUCCESS;
        as_->setSucceeded(result,"empty path");
        return;
    }

    std::vector<geometry_msgs::PoseStamped> final_path;
    final_path.push_back(follow_path_goal_->target_path.poses.at(0));
    final_path.at(0).header = header;
    ROS_INFO_STREAM("path.header.frame_id="<<final_path.at(0).header.frame_id); //NB
    ROS_INFO_STREAM("path.header.stamp="<<final_path.at(0).header.stamp); //NB

    for(unsigned int i=1; i< (follow_path_goal_->target_path.poses.size()-1); i++) //den ersten Speichern wir auf jeden fall ab deswegen i =1 , den letzten speichern wir auch auf jeden fall deswege  size()-1
    {
        geometry_msgs::PoseStamped p;
        p.pose = follow_path_goal_->target_path.poses.at(i).pose;
        p.header = header;
        float x = p.pose.position.x;
        float y = p.pose.position.y;
        if((x>= (2.5 + final_path.back().pose.position.x)) || (y>= (2.5 + final_path.back().pose.position.y)))  //wir wollen alle 2.5 meter einen neuen Punkt abspeichern
        {
            final_path.push_back(p);

        }

    }
    final_path.push_back(follow_path_goal_->target_path.poses.back()); // letzten Eintrag noch abspeichern
    final_path.back().header = header;
    //nav_msgs::Path::ConstPtr& path2use = &(follow_path_goal_->target_path);
    nav_msgs::Path path2use = follow_path_goal_->target_path;
    path2use.poses.clear();
    for (size_t i = 0; i < final_path.size(); ++i) {
        path2use.poses.push_back(final_path[i]);
    }

    ROS_INFO("FollowPath");
    planner.setPlan(final_path);
    planner.customViaPointsCB(path2use);
    //CB_via_points(path2use);
    planSet = true;


    
    

}  //NB1

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

}  //NB1



/*// Planning loop
void CB_mainCycle(const ros::TimerEvent& e)
{
  planner->plan(PoseSE2(-4,0,0), PoseSE2(4,0,0)); // hardcoded start and goal for testing purposes
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent& e)
{
  planner->visualize();
  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);
}

void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
  config.reconfigure(reconfig);
}

void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;
  i_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.id = id;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  box_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  i_marker.controls.push_back( box_control );

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name,feedback_cb);
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;
  
  if (index>=no_fixed_obstacles) 
    return;
  PointObstacle* pobst = static_cast<PointObstacle*>(obst_vector.at(index).get());
  pobst->position() = Eigen::Vector2d(feedback->pose.position.x,feedback->pose.position.y);	  
}

void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);
  
  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)  
  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
  {
    if (obst_msg->obstacles.at(i).polygon.points.size() == 1 )
    {
      if (obst_msg->obstacles.at(i).radius == 0) 
      {
        obst_vector.push_back(ObstaclePtr(new PointObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                           obst_msg->obstacles.at(i).polygon.points.front().y )));
      }
      else
      {
        obst_vector.push_back(ObstaclePtr(new CircularObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                            obst_msg->obstacles.at(i).polygon.points.front().y,
                                                            obst_msg->obstacles.at(i).radius )));
      }
    }
    else if (obst_msg->obstacles.at(i).polygon.points.empty())
    {
      ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
      continue;
    }
    else
    {
      PolygonObstacle* polyobst = new PolygonObstacle;
      for (size_t j=0; j<obst_msg->obstacles.at(i).polygon.points.size(); ++j)
      {
        polyobst->pushBackVertex( obst_msg->obstacles.at(i).polygon.points[j].x,
                                  obst_msg->obstacles.at(i).polygon.points[j].y );
      }
      polyobst->finalizePolygon();
      obst_vector.push_back(ObstaclePtr(polyobst));
    }

    if(!obst_vector.empty())
      obst_vector.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
  }
}


void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg)
{
  // we assume for simplicity that the fixed frame is already the map/planning frame
  // consider clicked points as via-points
  via_points.push_back( Eigen::Vector2d(point_msg->point.x, point_msg->point.y) );
  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
  if (config.optim.weight_viapoint<=0)
    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
}

void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}

void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id)
{
  if (id >= obst_vector.size())
  {
    ROS_WARN("Cannot set velocity: unknown obstacle id.");
    return;
  }

  Eigen::Vector2d vel (twist_msg->linear.x, twist_msg->linear.y);
  obst_vector.at(id)->setCentroidVelocity(vel);
}*/
