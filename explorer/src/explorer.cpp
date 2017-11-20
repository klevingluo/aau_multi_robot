#include "ExplorationPlanner.h"
#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <ExplorationPlanner.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/observation.h>
#include <costmap_2d/observation_buffer.h>
#include <iostream>
#include <map_merger/LogMaps.h>
#include <move_base/MoveBaseConfig.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <navfn/navfn_ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

class Explorer {

  public:

    Explorer(tf::TransformListener& tf) :
      counter(0), 
      nh("~"), 
      exploration_finished(false), 
      number_of_robots(1), 
      cluster_element(-1), 
      cluster_initialize_flag(false), 
      global_costmap_iteration(0), 
      robot_prefix_empty(false), 
      robot_id(0){

        nh.param("frontier_selection",frontier_selection,1); 
        nh.param("local_costmap/width",costmap_width,0); 
        nh.param<double>("local_costmap/resolution",costmap_resolution,0);
        nh.param("number_unreachable_for_cluster", number_unreachable_frontiers_for_cluster,3);
        nh.param<std::string>("move_base_frame",move_base_frame,"map");  
        nh.param<int>("wait_for_planner_result",waitForResult,3);
        nh.param<std::string>("robot_prefix",robot_prefix,"");

        srand((unsigned)time(0));

        // create map_merger service
        std::string service = robot_prefix + std::string("/map_merger/logOutput");
        mm_log_client = nh.serviceClient<map_merger::LogMaps>(service.c_str());

        if(robot_prefix.empty())
        {
          char hostname_c[1024];
          hostname_c[1023] = '\0';
          gethostname(hostname_c, 1023);
          robot_name = std::string(hostname_c);
          ROS_INFO("NO SIMULATION! Robot name: %s", robot_name.c_str());

          /*
           * THIS IS REQUIRED TO PERFORM COORDINATED EXPLORATION
           * Assign numbers to robot host names in order to make
           * auctioning and frontier selection UNIQUE !!!!!
           * To use explorer node on a real robot system, add your robot names 
           * here and at ExplorationPlanner::lookupRobotName function ... 
           */
          std::string bob = "bob";
          std::string marley = "marley";
          std::string turtlebot = "turtlebot";
          std::string joy = "joy";
          std::string hans = "hans";

          if(robot_name.compare(turtlebot) == 0)
            robot_id = 0;
          if(robot_name.compare(joy) == 0)
            robot_id = 1;
          if(robot_name.compare(marley) == 0)
            robot_id = 2;
          if(robot_name.compare(bob) == 0)
            robot_id = 3;
          if(robot_name.compare(hans) == 0)
            robot_id = 4;

          robot_prefix_empty = true;
          ROS_INFO("Robot name: %s    robot_id: %d", robot_name.c_str(), robot_id);
        } else
        {
          robot_name = robot_prefix;
          ROS_INFO("Move_base_frame: %s",move_base_frame.c_str());               
          robot_id = atoi(move_base_frame.substr(7,1).c_str());

          ROS_INFO("Robot: %d", robot_id);
        } 
        initLogPath();

        visualize_goal_point(
            robotPose.getOrigin().getX(),
            robotPose.getOrigin().getY());

        exploration = new explorationPlanner::ExplorationPlanner(robot_id, robot_prefix_empty, robot_name);

        localCostmapSub = nh.subscribe(
            robot_prefix + "move_base/local_costmap/costmap", 
            1, 
            &Explorer::localCostmapCallback,
            this);

        globalCostmapSub = nh.subscribe(
            robot_prefix + "merged_map",  //TODO:  fix this
            1, 
            &Explorer::globalCostmapCallback,
            this);

        odomSub = nh.subscribe(
            robot_prefix + "odom", 
            1, 
            &Explorer::odomCallback,
            this);

        while(ros::ok()) {
          ros::spinOnce();
          ROS_ERROR("No map_meta information");
          ros::Duration(0.3).sleep();
          if(exploration->initialized_planner)
            break;
        }

        robot_home_position_x = robotPose.getOrigin().getX();
        robot_home_position_y = robotPose.getOrigin().getY();

        exploration->next_auction_position_x = robotPose.getOrigin().getX();
        exploration->next_auction_position_y = robotPose.getOrigin().getY();

        exploration->storeVisitedFrontier(robot_home_position_x,robot_home_position_y, robot_id, robot_name, -1);		             

        exploration->setRobotConfig(robot_id, robot_home_position_x, robot_home_position_y, move_base_frame);
      }

    /**
     * explore until no frontiers remain
     */
    void explore() 
    {
      time_start = ros::Time::now();

      while (exploration_finished == false) 
      {
        std::vector<double> final_goal;
        std::vector<std::string> robot_str;

        bool negotiation;
        int count = 0;

        exploration->findFrontiers();
        exploration->clearUnreachableFrontiers();
        exploration->clusterFrontiers();
        //exploration->visualize_Frontiers();
        exploration->visualizeClustersConsole();

        /*********** EXPLORATION STRATEGY ************
         * 0 ... Navigate to nearest frontier TRAVEL PATH
         * 1 ... Navigate using auctioning with cluster selection using 
         *       NEAREST selection (Kuhn-Munkres)
         * 5 ... Cluster frontiers, then navigate to nearest cluster 
         *       using EUCLIDEAN DISTANCE (with and without
         *       negotiation)
         */

        /******************** SORT *******************
         * Choose which strategy to take.
         * 1 ... Sort the buffer from furthest to nearest frontier
         * 2 ... Sort the buffer from nearest to furthest frontier, normalized to the
         *       robots actual position (EUCLIDEAN DISTANCE)
         * 3 ... Sort the last 10 entries to shortest TRAVEL PATH
         * 4 ... Sort all cluster elements from nearest to furthest (EUCLIDEAN DISTANCE)
         */
        frontier_selection = 6;
        if(frontier_selection < 0 || frontier_selection > 6)
        {
          ROS_FATAL("You selected an invalid exploration strategy. Please make sure to set it in the interval [0,6]. The current value is %d.", frontier_selection);
        } else if (true)
        {
          ROS_WARN("planning goal");
          goal_determined = exploration->determine_goal(
              6, 
              &final_goal, 
              0, 
              cluster_element, 
              &robot_str);
        }
        else if(frontier_selection == 0)
        {
          exploration->sort(2);
          goal_determined = exploration->determine_goal(2, &final_goal, count, 0, &robot_str);
          ROS_DEBUG("Goal_determined: %d counter: %d",goal_determined, count);
        }
        else if(frontier_selection == 5)
        {                            
          exploration->sort(2); 
          exploration->clusterFrontiers();                 
          exploration->sort(4);      

          clusters_available_in_pool.clear();
          goal_determined = exploration->determine_goal(
              5, 
              &final_goal, 
              count, 
              cluster_element, &robot_str);
        } 
        else if(frontier_selection == 1)
        {     
          exploration->clusterFrontiers();                 
          exploration->sort(4);      
          goal_determined = exploration->determine_goal(1, &final_goal, 0, cluster_element, &robot_str);
        } 
        else if(frontier_selection = 6) 
        {
          goal_determined = exploration->determine_goal(
              6, 
              &final_goal, 
              0, 
              cluster_element, 
              &robot_str);
        }
        ROS_WARN("Goal_determined: %d counter: %d",goal_determined, count);

        if(goal_determined)
        {
          ROS_INFO("Doing navigation to goal");
          if(navigate(final_goal))
          {
            exploration->calculate_travel_path(
                exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).x_coordinate, 
                exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).y_coordinate);
          } else 
          {
            ROS_WARN("failed to navigate: Storing unreachable frontier with id:");
          } 
          exploration->storeUnreachableFrontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),robot_str.at(0),final_goal.at(3)); 
        }
      }
    }

    void initLogPath()
    {
      /*
       *  CREATE LOG PATH
       * Following code enables to write the output to a file
       * which is localized at the log_path
       */

      ROS_INFO("debug initlogpath");
      nh.param<std::string>("log_path",log_path,"");           

      std::stringstream robot_number;
      robot_number << robot_id;
      std::string prefix = "/robot_";
      std::string robo_name = prefix.append(robot_number.str());    

      log_path = log_path.append("/explorer");
      log_path = log_path.append(robo_name);
      log_path = log_path.append("/");
      ROS_INFO("Logging files to %s", log_path.c_str());

      boost::filesystem::path boost_log_path(log_path.c_str());
      if(!boost::filesystem::exists(boost_log_path))
        if(!boost::filesystem::create_directories(boost_log_path))
          // directory not created; check if it is there anyways
          if(!boost::filesystem::is_directory(boost_log_path))
            ROS_ERROR("Cannot create directory %s.", log_path.c_str());
          else
            ROS_INFO("Successfully created directory %s.", log_path.c_str());

    }

    /*
     * If received goal is not empty (x=0 y=0), drive the robot to this point
     * and mark that goal as seen in the last_goal_position vector!!!
     * Otherwise turn the robot by 90° to the right and search again for a better
     * frontier.
     */
    bool navigate(std::vector<double> goal) {
      visualize_goal_point(goal.at(0), goal.at(1));

      actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> ac("move_base", true);

      while(!ac.waitForServer(ros::Duration(5.0)))
        ROS_INFO("Waiting for the move_base action server to come up");

      move_base_msgs::MoveBaseGoal navGoal;

      navGoal.target_pose.header.seq = seq;	// increase the sequence number
      navGoal.target_pose.header.stamp = ros::Time::now();
      navGoal.target_pose.header.frame_id = move_base_frame; //"map";
      navGoal.target_pose.pose.position.x = goal.at(0);// - robotPose.getOrigin().getX(); //navGoals[0].pose.position.x;
      navGoal.target_pose.pose.position.y = goal.at(1);// - robotPose.getOrigin().getY();
      navGoal.target_pose.pose.orientation.w = 1.0;

      tf::Matrix3x3 m(1,0,0,0,1,0,0,0,1);
      tf::Quaternion q;

      m.setRPY(0,0,3.14/2);
      m.getRotation(q);
      navGoal.target_pose.pose.orientation.x = q.getX();
      navGoal.target_pose.pose.orientation.y = q.getY();
      navGoal.target_pose.pose.orientation.z = q.getZ();
      navGoal.target_pose.pose.orientation.w = q.getW();
      ac.sendGoalAndWait(navGoal, ros::Duration(20));

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_WARN("navigation failed:");
        return true;
      }
      return false;
    }

    void visualize_goal_point(double x, double y) {

      goalPoint.header.seq = goal_point_message++;
      goalPoint.header.stamp = ros::Time::now();
      goalPoint.header.frame_id = move_base_frame; //"map"
      goalPoint.point.x = x;// - robotPose.getOrigin().getX();
      goalPoint.point.y = y;// - robotPose.getOrigin().getY();

      ros::NodeHandle nh_Point("goalPoint");
      pub_Point = nh_Point.advertise < geometry_msgs::PointStamped
        > ("goalPoint", 100, true);
      pub_Point.publish < geometry_msgs::PointStamped > (goalPoint);
    }

    void feedbackCallback(
        const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
      feedback_value = msg.get()->status.status;
      feedback_succeed_value =
        msg.get()->feedback.base_position.pose.orientation.w;

    }

    void localCostmapCallback(nav_msgs::OccupancyGridPtr msg) {
      nav_msgs::OccupancyGrid costmap = *msg.get();
      exploration->updateLocalCostmap(&costmap);
    }

    void odomCallback(nav_msgs::OdometryPtr msg) {
      nav_msgs::Odometry odom = *msg.get();
      exploration->updateOdom(&odom);
    }

    void globalCostmapCallback(nav_msgs::OccupancyGridPtr msg) {
      nav_msgs::OccupancyGrid costmap = *msg.get();
      exploration->updateGlobalCostmap(&costmap);
    }

  public:

    struct map_progress_t
    {
      double local_freespace;
      double global_freespace;
      double time;
    } map_progress;

    ros::Subscriber globalCostmapSub;
    ros::Subscriber localCostmapSub;
    ros::Subscriber odomSub;

    // create a costmap
    costmap_2d::Costmap2D costmap;

    std::vector<map_progress_t> map_progress_during_exploration;

    std::vector<int> clusters_available_in_pool;

    int home_position_x, home_position_y;
    int robot_id, number_of_robots;
    int frontier_selection, costmap_width, global_costmap_iteration, number_unreachable_frontiers_for_cluster;

    double robot_home_position_x, robot_home_position_y, costmap_resolution;
    bool goal_determined;
    bool robot_prefix_empty;
    int cluster_element_size, cluster_element;
    bool cluster_initialize_flag;
    int waitForResult;
    std::string move_base_frame;
    std::string robot_prefix;
    std::string robot_name;
    const unsigned char* occupancy_grid_global;
    const unsigned char* occupancy_grid_local;

    std::string log_path;
    std::fstream fs_csv, fs;

  private:

    ros::Publisher pub_move_base;
    ros::Publisher pub_Point;
    ros::Publisher pub_home_Point;
    ros::Publisher pub_frontiers;

    ros::ServiceClient mm_log_client;

    ros::NodeHandle nh;
    ros::Time time_start;

    move_base_msgs::MoveBaseActionGoal action_goal_msg;
    move_base_msgs::MoveBaseActionFeedback feedback_msgs;

    geometry_msgs::PointStamped goalPoint;
    geometry_msgs::PointStamped homePoint;

    std::vector<geometry_msgs::PoseStamped> goals;
    tf::Stamped<tf::Pose> robotPose;

    explorationPlanner::ExplorationPlanner *exploration;       

    double x_val, y_val, home_point_x, home_point_y;
    int seq, feedback_value, feedback_succeed_value, 
        home_point_message, goal_point_message;
    int counter;
    bool pioneer, exploration_finished;
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "simple_navigation");

  // Create instance of Simple Navigation
  tf::TransformListener tf(ros::Duration(10));
  Explorer simple(tf);

  boost::thread thr_explore(boost::bind(&Explorer::explore, &simple));	

  ros::spin();
  return 0;
}
