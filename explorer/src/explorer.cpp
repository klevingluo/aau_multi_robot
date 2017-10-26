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

        ROS_INFO("Costmap width: %d", costmap_width);
        ROS_INFO("Frontier selection is set to: %d", frontier_selection);
        Simulation = false;      

        srand((unsigned)time(0));

        nh.param<std::string>("move_base_frame",move_base_frame,"map");  
        nh.param<int>("wait_for_planner_result",waitForResult,3);
        nh.param<std::string>("robot_prefix",robot_prefix,"");

        ROS_INFO("robot prefix: \"%s\"", robot_prefix.c_str());

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

        /*
         *  CREATE LOG PATH
         * Following code enables to write the output to a file
         * which is localized at the log_path
         */
        initLogPath();
        csv_file = log_path + std::string("periodical.log");
        log_file = log_path + std::string("exploration.log"); 

        visualize_goal_point(robotPose.getOrigin().getX(),
            robotPose.getOrigin().getY());

        exploration = new explorationPlanner::ExplorationPlanner(robot_id, robot_prefix_empty, robot_name);

        localCostmapSub = nh.subscribe(
            "/robot_1/move_base/local_costmap/costmap", 
            1, 
            &Explorer::localCostmapCallback,
            this);

        globalCostmapSub = nh.subscribe(
            "/robot_1/map", 
            1, 
            &Explorer::globalCostmapCallback,
            this);

        odomSub = nh.subscribe(
            "/robot_1/odom", 
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

        /*
         * Define the first Goal. This is required to have at least one entry
         * within the vector. Therefore set it to the home position.
         */

        robot_home_position_x = robotPose.getOrigin().getX();
        robot_home_position_y = robotPose.getOrigin().getY();
        ROS_INFO("Set home point to (%lf,%lf).",robot_home_position_x,robot_home_position_y);

        exploration->next_auction_position_x = robotPose.getOrigin().getX();
        exploration->next_auction_position_y = robotPose.getOrigin().getY();

        exploration->storeVisitedFrontier(robot_home_position_x,robot_home_position_y, robot_id, robot_name, -1);		             
        // on't need this?
        //exploration->storeFrontier(robot_home_position_x,robot_home_position_y, robot_id, robot_name, -1);

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
        ROS_WARN("calling explore");
        frontier_selection = 6;
        if(frontier_selection < 0 || frontier_selection > 6)
        {
          ROS_FATAL("You selected an invalid exploration strategy. Please make sure to set it in the interval [0,6]. The current value is %d.", frontier_selection);
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
        ROS_DEBUG("Goal_determined: %d counter: %d",goal_determined, count);

        if(goal_determined)
        {
          ROS_INFO("Doing navigation to goal");
          if(navigate(final_goal))
          {
            exploration->calculate_travel_path(exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).x_coordinate, exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).y_coordinate);
          } else 
          {
            ROS_WARN("failed to navigate: Storing unreachable frontier with id:");
            exploration->storeUnreachableFrontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),robot_str.at(0),final_goal.at(3)); 
          } 
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

    void save_progress(bool final=false)
    {
      ROS_INFO("save_progres");
      ros::Duration ros_time = ros::Time::now() - time_start;

      double exploration_time = ros_time.toSec();           
      int navigation_goals_required = counter;        
      double exploration_travel_path = (double)exploration->exploration_travel_path_global * costmap_resolution;
      double size_global_map = map_progress_during_exploration.at(map_progress_during_exploration.size()-1).global_freespace;

      double efficiency_value = (exploration_time) / (number_of_robots * navigation_goals_required);

      std::string tmp_log;
      if(!final) 
      {
        tmp_log = log_file + std::string(".tmp");
        fs.open(tmp_log.c_str(), std::fstream::in | std::fstream::trunc | std::fstream::out);
      }
      else
      {
        tmp_log = log_file;
        fs.open(tmp_log.c_str(), std::fstream::in | std::fstream::trunc | std::fstream::out);
      }

      /*
       *  WRITE LOG FILE
       * Write all the output to the log file
       */
      time_t raw_time;
      struct tm* timeinfo;
      time (&raw_time);
      timeinfo = localtime (&raw_time);

      fs << "[Exploration]" << std::endl;
      fs << "time_file_written    			= " << asctime(timeinfo); // << std::endl;
      fs << "start_time           			= " << time_start << std::endl;
      fs << "end_time             			= " << ros::Time::now() << std::endl;
      fs << "exploration_time    			        = " << exploration_time << std::endl;
      fs << "required_goals       			= " << navigation_goals_required << std::endl;  
      fs << "unreachable_goals                            = " << exploration->unreachable_frontiers.size() << std::endl;
      fs << "travel_path_overall  			= " << exploration_travel_path << std::endl;
      fs << "number_of_completed_auctions                 = " << exploration->number_of_completed_auctions << std::endl;
      fs << "number_of_uncompleted_auctions               = " << exploration->number_of_uncompleted_auctions << std::endl;
      fs << "frontier_selection_strategy                  = " << frontier_selection << std::endl;
      fs << "costmap_size                                 = " << costmap_width << std::endl;
      fs << "global costmap iterations                    = " << global_costmap_iteration << std::endl;

      double param_double;
      int param_int;
      std::string param;
      /*param = robot_prefix + "/explorer/local_costmap/height";
        ros::param::get(param,param_double);*/
      nh.param<int>("local_costmap/height",param_int,-1);
      fs << "explorer_local_costmap_height 		= " << param_int << std::endl;

      /*param = robot_prefix + "/explorer/local_costmap/width";
        ros::param::get(param,param_double);*/
      nh.param<int>("local_costmap/width",param_int,-1);
      fs << "explorer_local_costmap_width 		= " << param_int << std::endl;

      param = robot_prefix + "/move_base/local_costmap/height";
      ros::param::get(param,param_double);
      fs << "move_base_local_costmap_height 		= " << param_double << std::endl;

      param = robot_prefix + "/move_base/local_costmap/width";
      ros::param::get(param,param_double);
      fs << "move_base_local_costmap_width 		= " << param_double << std::endl;

      param = robot_prefix + "/move_base/global_costmap/obstacle_layer/raytrace_range";
      ros::param::get(param,param_double);
      fs << "move_base_raytrace_range 		= " << param_double << std::endl;

      param = robot_prefix + "/move_base/global_costmap/obstacle_layer/obstacle_range";
      ros::param::get(param,param_double);
      fs << "move_base_obstacle_range 		= " << param_double << std::endl;

      //	    param = robot_prefix + "/navigation/global_costmap/obstacle_layer/raytrace_range";
      nh.getParam("/global_costmap/obstacle_layer/raytrace_range",param_double);
      ros::param::get(param,param_double);
      fs << "explorer_raytrace_range 		= " << param_double << std::endl;

      //param = robot_prefix + "/navigation/global_costmap/obstacle_layer/obstacle_range";
      //    ros::param::get(param,param_double);
      nh.getParam("/global_costmap/obstacle_layer/obstacle_range",param_double);
      fs << "explorer_obstacle_range 		= " << param_double << std::endl;

      if(final)
        fs << "complete             			= " << "1" << std::endl;
      else
        fs << "complete             			= " << "0" << std::endl;

      fs.close();
      /*
       * Inform map_merger to save maps
       */

      if(final)
      {
        map_merger::LogMaps log;
        log.request.log = 3;    /// request local and global map
        if(!mm_log_client.call(log))
          ROS_WARN("Could not call map_merger service to store log.");
      }

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

      for (int i = 0; i < 4; i++)
      {
        m.setRPY(0,0,i*3.14/2);
        m.getRotation(q);
        navGoal.target_pose.pose.orientation.x = q.getX();
        navGoal.target_pose.pose.orientation.y = q.getY();
        navGoal.target_pose.pose.orientation.z = q.getZ();
        navGoal.target_pose.pose.orientation.w = q.getW();
        ac.sendGoal(navGoal);
        ac.waitForResult();
      }

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_WARN("navigation failed:");
        return true;
      }
      return false;
    }

    void visualize_goal_point(double x, double y) {
      ROS_INFO("nav");

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

    //returns true if the action succeeded, false on abort, 
    bool move_robot(int seq, double position_x, double position_y) {
      return false;
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
    bool Simulation, goal_determined;
    bool robot_prefix_empty;
    int cluster_element_size, cluster_element;
    bool cluster_initialize_flag;
    int waitForResult;
    std::string move_base_frame;
    std::string robot_prefix;
    std::string robot_name;
    const unsigned char* occupancy_grid_global;
    const unsigned char* occupancy_grid_local;

    std::string csv_file, log_file;
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

  while (ros::ok()) {
    ros::spinOnce();

    ros::Duration(0.1).sleep();
  }

  thr_explore.interrupt();

  thr_explore.join();

  return 0;
}
