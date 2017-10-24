#include "ros/ros.h"
#include "ExplorationPlanner.h"
#include <ros/console.h>
#include <ExplorationPlanner.h>
#include <boost/lexical_cast.hpp>
#include <move_base/MoveBaseConfig.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/observation.h>
#include <costmap_2d/observation_buffer.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <navfn/navfn_ros.h>
#include <boost/filesystem.hpp>
#include <map_merger/LogMaps.h>

boost::mutex costmap_mutex;

class Explorer {

  public:

    Explorer(tf::TransformListener& tf) :
      counter(0), 
      rotation_counter(0), 
      nh("~"), 
      exploration_finished(false), 
      number_of_robots(1), 
      accessing_cluster(0), 
      cluster_element_size(0),
      cluster_flag(false), 
      cluster_element(-1), 
      cluster_initialize_flag(false), 
      global_iterattions(0), 
      global_iterations_counter(0), 
      counter_waiting_for_clusters(0), 
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
        // determine host name
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

        ROS_DEBUG("Creating global costmap ...");
        costmap2d_global = new costmap_2d::Costmap2DROS("global_costmap", tf);
        ROS_DEBUG("Global costmap created ... now performing costmap -> pause");
        costmap2d_global->pause();
        ROS_DEBUG("Pausing performed");

        costmap2d_local_size = new costmap_2d::Costmap2DROS("local_costmap", tf);
        costmap2d_local_size->pause();
        ROS_DEBUG("Starting Global costmap ...");
        costmap2d_global->start();
        costmap2d_local_size->start();

        costmap2d_local = costmap2d_global;                   

        if (!costmap2d_local->getRobotPose(robotPose)) {
          ROS_ERROR("Failed to get RobotPose");
        }
        visualize_goal_point(robotPose.getOrigin().getX(),
            robotPose.getOrigin().getY());

        // instantiate the planner
        exploration = new explorationPlanner::ExplorationPlanner(robot_id, robot_prefix_empty, robot_name);

        ros::Subscriber localCostmapSub = nh.subscribe(
            "/robot_1/move_base/local_costmap/costmap", 
            1, 
            &Explorer::localCostmapCallback,
            this);

        ros::Subscriber globalCostmapSub = nh.subscribe(
            "/robot_1/move_base/global_costmap/costmap", 
            1, 
            &Explorer::globalCostmapCallback,
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
        exploration->storeFrontier(robot_home_position_x,robot_home_position_y, robot_id, robot_name, -1);           

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

        bool navigation_successful = false;
        bool negotiation;
        int count = 0;

        /*
         * Use mutex to lock the critical section 
         * (access to the costmap)
         * since rosspin tries to update the costmap continuously
         */
        costmap_mutex.lock();

        exploration->transformToOwnCoordinates_frontiers();
        exploration->transformToOwnCoordinates_visited_frontiers();

        exploration->findFrontiers();

        // adds unvisited frontiers to a list, comparing them to a list of seen frontiers
        exploration->clearVisitedFrontiers();                       
        // same, but with unreachable frontiers
        exploration->clearUnreachableFrontiers();
        // dunno probably just checks the costmap for the frontiers
        exploration->clearSeenFrontiers();       

        // this doesn't do anything, right?
        costmap_mutex.unlock();

        // as in sends them out on multicast
        exploration->publish_frontier_list();  
        exploration->publish_visited_frontier_list();  

        ros::Duration(1).sleep();
        exploration->publish_frontier_list();  
        exploration->publish_visited_frontier_list();  

        ros::Duration(1).sleep();

        //FRONTIER SELECTION

        /*********** EXPLORATION STRATEGY ************
         * 0 ... Navigate to nearest frontier TRAVEL PATH
         * 1 ... Navigate using auctioning with cluster selection using 
         *       NEAREST selection (Kuhn-Munkres)
         * 2 ... Navigate to furthest frontier
         * 3 ... Navigate to nearest frontier EUCLIDEAN DISTANCE
         * 4 ... Navigate to random Frontier
         * 5 ... Cluster frontiers, then navigate to nearest cluster 
         *       using EUCLIDEAN DISTANCE (with and without
         *       negotiation)
         * 6 ... Cluster frontiers, then navigate to random cluster
         *       (with and without negotiation)
         */

        /******************** SORT *******************
         * Choose which strategy to take.
         * 1 ... Sort the buffer from furthest to nearest frontier
         * 2 ... Sort the buffer from nearest to furthest frontier, normalized to the
         *       robots actual position (EUCLIDEAN DISTANCE)
         * 3 ... Sort the last 10 entries to shortest TRAVEL PATH
         * 4 ... Sort all cluster elements from nearest to furthest (EUCLIDEAN DISTANCE)
         */
        if(frontier_selection < 0 || frontier_selection > 6)
        {
          ROS_FATAL("You selected an invalid exploration strategy. Please make sure to set it in the interval [0,6]. The current value is %d.", frontier_selection);
        }
        if(frontier_selection == 2)
        {

          exploration->sort(1); 

          /* 
           * Choose which strategy to take.
           * 1 ... least to first goal in list
           * 2 ... first to last goal in list
           *
           * Determine_goal() takes world coordinates stored in
           * the frontiers list(which contain every found frontier with sufficient
           * spacing in between) and returns the most attractive one, based on the
           * above defined strategies.
           */    
          while(true)
          {   
            if(goal_determined == false)
            {
              break;
            }
            else
            {   
              //negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
              negotiation = true; 
              if(negotiation == true)
              {
                break;
              }
              count++;
            }
          }
        }    
        else if(frontier_selection == 3)
        {                           
          exploration->sort(2);

          while(true)
          {   
            goal_determined = exploration->determine_goal(2, &final_goal, count, 0, &robot_str);
            if(goal_determined == false)
            {
              break;
            }
            else
            {
              //negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
              negotiation = true; 
              if(negotiation == true)
              {
                break;
              }
              count++;
            }
          }
        }        
        else if(frontier_selection == 0)
        {
          exploration->sort(2);

          while(true)
          {   
            goal_determined = exploration->determine_goal(2, &final_goal, count, 0, &robot_str);
            ROS_DEBUG("Goal_determined: %d counter: %d",goal_determined, count);
            if(goal_determined == false)
            {
              break;
            }
            else
            {                        
              //negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
              negotiation = true;
              break;
              count++;
            }
          }
        }
        else if(frontier_selection == 4)
        {
          while(true)
          {   
            goal_determined = exploration->determine_goal(3, &final_goal, count, 0, &robot_str);
            ROS_DEBUG("Goal_determined: %d   counter: %d",goal_determined, count);
            if(goal_determined == false)
            {
              break;
            }
            else
            {
              //negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),-1);
              negotiation = true; 
              if(negotiation == true)
              {
                break;
              }
              count++;
            }
          }

        }
        else if(frontier_selection == 5)
        {                            
          if(cluster_initialize_flag == true)
          {
            exploration->clearVisitedAndSeenFrontiersFromClusters();
          }
          else
          {
            /*
             * This is necessary for the first run 
             */
            if(robot_id == 0)
              ros::Duration(10).sleep();

            exploration->sort(2); 
          }

          exploration->clusterFrontiers();                 
          exploration->sort(4);      
          exploration->sort(5);

          //                            exploration->visualizeClustersConsole();

          clusters_available_in_pool.clear();
          while(true)
          {                             
            final_goal.clear();
            goal_determined = exploration->determine_goal(4, &final_goal, count, cluster_element, &robot_str);

            if(goal_determined == false)
            {
              ROS_INFO("Another cluster is not available, no cluster determined");                                    
              break;
            }
            else
            {
              if(cluster_initialize_flag == false)
              {
                /*
                 *  TODO ... just to reduce the selection of the same 
                 * clusters since no auctioning is implemented jet.
                 */
                if(robot_id == 1)  
                {
                  ros::Duration(5).sleep();
                }

                cluster_initialize_flag = true;
              }

              /*
               * If negotiation is not needed, simply uncomment
               * and set the negotiation to TRUE.
               */
              negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),final_goal.at(4));
              //                                        negotiation = true; 
              if(negotiation == true)
              {
                ROS_DEBUG("Negotiation was successful");
                cluster_element = final_goal.at(4);
                counter_waiting_for_clusters = 0;
                break;
              }
              else
              {
                cluster_element = final_goal.at(4);
                counter_waiting_for_clusters++;
                ROS_WARN("Negotiation was not successful, try next cluster");
              }
              count++;
              /*
               * In order to make one robot wait until the other finds
               * another cluster to operate in, one have to fill the clusters_available_in_pool
               * vector if count is increased at least once which determines
               * that at least one cluster
               */
              clusters_available_in_pool.push_back(1);
            }
          }
        } 
        else if(frontier_selection == 6)
        {                            
          if(cluster_initialize_flag == true)
          {
            exploration->clearVisitedAndSeenFrontiersFromClusters();
          }

          exploration->clusterFrontiers();   
          exploration->sort(6);
          exploration->visualizeClustersConsole();

          if(cluster_initialize_flag == false)
          {
            ros::Duration(2).sleep();
          }
          cluster_initialize_flag = true;

          while(true)
          {             
            goal_determined = exploration->determine_goal(4, &final_goal, count, cluster_element, &robot_str);

            if(goal_determined == false)
            {
              break;
            }
            else
            {   
              /*
               * If negotiation is not needed, simply uncomment
               * and set the negotiation to TRUE.
               */
              negotiation = exploration->negotiate_Frontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),final_goal.at(3),final_goal.at(4));
              //                                        negotiation = true; 

              if(negotiation == true)
              {
                cluster_element = final_goal.at(4);
                break;
              }
              else
              {
                cluster_element = int(exploration->clusters.size()*rand()/(RAND_MAX));
                ROS_INFO("Random cluster_element: %d  from available %lu clusters", cluster_element, exploration->clusters.size());
              }
              count++;
            }
          }
        } 
        else if(frontier_selection == 1)
        {     
          costmap_mutex.lock();
          if(cluster_initialize_flag == true)
          {
            exploration->clearVisitedAndSeenFrontiersFromClusters();
          }
          else
          {                              
            /*
             * This is only necessary for the first run 
             */

            exploration->sort(2); 
            cluster_initialize_flag = true;
          }

          exploration->clusterFrontiers();                 
          exploration->sort(4);      
          exploration->sort(5);

          costmap_mutex.unlock();

          exploration->visualizeClustersConsole();

          while(true)
          {                             
            final_goal.clear();
            robot_str.clear();
            goal_determined = exploration->determine_goal(5, &final_goal, 0, cluster_element, &robot_str);

            ROS_DEBUG("Cluster element: %d", cluster_element);

            int cluster_vector_position = -1;

            // find index of element with id cluster_element
            if(cluster_element != -1)
            {
              if(exploration->clusters.size() > 0)
              {                   
                for (int i = 0; i < exploration->clusters.size(); i++)
                {
                  if(exploration->clusters.at(i).id == cluster_element)
                  {
                    if(exploration->clusters.at(i).cluster_element.size() > 0)
                    {
                      cluster_vector_position = i; 
                    }
                    break;
                  }
                }
              }
            }
            ROS_DEBUG("Cluster vector position: %d", cluster_vector_position);

            if(cluster_vector_position >= 0)
            {
              if(exploration->clusters.at(cluster_vector_position).unreachable_frontier_count >= number_unreachable_frontiers_for_cluster)
              {
                goal_determined = false;
                ROS_WARN(
                    "too many unreacheachable frontiers in cluster: %i: max: %i", 
                    exploration->clusters.at(cluster_vector_position).unreachable_frontier_count, 
                    number_unreachable_frontiers_for_cluster);
              } else
              {
                ROS_DEBUG("Cluster operateable");
              }
            }

            if(goal_determined == false)
            {  

              ROS_INFO("No goal was determined, cluster is empty. Bid for another one");

              final_goal.clear();
              robot_str.clear();
              clusters_available_in_pool.clear();

              bool auctioning = exploration->auctioning(&final_goal, &clusters_available_in_pool, &robot_str);
              if(auctioning == true)
              {
                goal_determined = true; 
                cluster_element = final_goal.at(4);
                counter_waiting_for_clusters = 0;
                break;
              }
              else
              {
                if(exploration->clusters.size() > 0) //clusters_available_in_pool.size() > 0)
                {
                  ROS_INFO("No cluster was selected but other robots are operating ... waiting for new clusters");
                  counter_waiting_for_clusters++;
                  break;
                }else
                {
                  /*
                   * If NO goals are selected at all, iterate over the global
                   * map to find some goals. 
                   */
                  ROS_WARN("No goals are available at all");
                  cluster_element = -1;
                  break;
                }
              }
            }
            else
            {
              ROS_INFO("Still some goals in current cluster, finish this job first");
              break;
            }
          }

        } 

        exploration->visualizeClustersConsole();
        exploration->visualize_Frontiers();

        if(goal_determined)
        {
          ROS_INFO("Doing navigation to goal");
          navigation_successful = navigate(final_goal);
          // successfully navigated to goal, marking frontier as visited
          // TODO: what about other frontier points that were scanned?
          if(navigation_successful)
          {
            exploration->calculate_travel_path(exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).x_coordinate, exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).y_coordinate);

            ROS_DEBUG("Storing visited...");
            exploration->storeVisitedFrontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),robot_str.at(0),final_goal.at(3)); 

          } else 
          {
            ROS_WARN("failed to navigate: Storing unreachable frontier with id:");
            exploration->storeUnreachableFrontier(final_goal.at(0),final_goal.at(1),final_goal.at(2),robot_str.at(0),final_goal.at(3)); 
            ROS_DEBUG("Stored unreachable frontier");
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

    void exploration_has_finished()
    {
      ROS_INFO("finished?");
      ros::Duration ros_time = ros::Time::now() - time_start;

      double exploration_time = ros_time.toSec();           
      int navigation_goals_required = counter;        
      double exploration_travel_path = (double)exploration->exploration_travel_path_global * 0.02;
      double size_global_map = map_progress_during_exploration.at(map_progress_during_exploration.size()-1).global_freespace;
      ROS_INFO("overall freespace in the global map: %f", size_global_map);

      for(int i = 0; i< map_progress_during_exploration.size(); i++)
      {
        ROS_INFO("map progress: %f",(map_progress_during_exploration.at(i).global_freespace/size_global_map)*100);
      }

      ROS_DEBUG("TIME: %f sec  GOALS: %d  PATH: %f  COMPLETED AUCTIONS: %d  UNCOMPLETED AUCTIONS: %d", exploration_time, navigation_goals_required, exploration_travel_path, exploration->number_of_completed_auctions, exploration->number_of_uncompleted_auctions);


      double efficiency_value = (exploration_time) / (number_of_robots * navigation_goals_required);

      /*
       *  WRITE LOG FILE
       * Write all the output to the log file
       */
      save_progress(true);
      ROS_INFO("Wrote file %s\n", log_file.c_str());                    

      /*
       * Inform map_merger to save maps
       */

      map_merger::LogMaps log;
      log.request.log = 3;    /// request local and global map
      if(!mm_log_client.call(log))
        ROS_WARN("Could not call map_merger service to store log.");
    }

    /**
     * finds frontiers on the global costmap
     */
    bool iterate_global_costmap(std::vector<double> *global_goal, std::vector<std::string> *robot_str)
    {
      global_costmap_iteration++;
      int counter = 0;
      bool exploration_flag;

      costmap_mutex.lock();

      exploration->transformToOwnCoordinates_frontiers();
      exploration->transformToOwnCoordinates_visited_frontiers();

      exploration->findFrontiers();
      exploration->visualize_Frontiers();

      exploration->clearVisitedFrontiers();                       
      exploration->clearUnreachableFrontiers();
      exploration->clearSeenFrontiers();       

      costmap_mutex.unlock();

      if(frontier_selection < 5 && frontier_selection != 1)
      {
        exploration->sort(2);

        while(true)
        {   
          exploration_flag = exploration->determine_goal(2, global_goal, counter, -1, robot_str);


          if(exploration_flag == false)
          {
            break;
          }
          else
          {    
            bool negotiation = true; 
            negotiation = exploration->negotiate_Frontier(global_goal->at(0),global_goal->at(1),global_goal->at(2),global_goal->at(3), -1);

            if(negotiation == true)
            {     
              return true;
              break;
            }           
            counter++;
          }
        }    
      }else if(frontier_selection == 1 || frontier_selection == 6 || frontier_selection == 5)
      {
        costmap_mutex.lock();
        exploration->clearVisitedAndSeenFrontiersFromClusters();

        exploration->clusterFrontiers(); 

        exploration->sort(4);      
        exploration->sort(5);

        costmap_mutex.unlock(); 

        cluster_element = -1;

        while(true)
        {                             
          std::vector<double> goal_vector;
          std::vector<std::string> robot_str_name;
          std::vector<int> clusters_used_by_others; 

          goal_determined = exploration->determine_goal(5, &goal_vector, 0, cluster_element, robot_str);

          if(goal_determined == false)
          {  
            ROS_INFO("No goal was determined, cluster is empty. Bid for another one");

            goal_vector.clear();
            bool auctioning = exploration->auctioning(&goal_vector, &clusters_used_by_others, &robot_str_name);
            if(auctioning == true)
            {                               
              goal_determined = true; 
              cluster_element = goal_vector.at(4);

              global_goal->push_back(goal_vector.at(0));
              global_goal->push_back(goal_vector.at(1));
              global_goal->push_back(goal_vector.at(2));
              global_goal->push_back(goal_vector.at(3));
              global_goal->push_back(goal_vector.at(4));

              robot_str->push_back(robot_str_name.at(0));
              return true; 
            }
            else
            {
              /*
               * If NO goals are selected at all, iterate over the global
               * map to find some goals. 
               */
              ROS_WARN("No goals are available at all");
              cluster_element = -1;
              break;
            }
          }
          else
          {
            ROS_INFO("Still some goals in current cluster, finish this job first");
            break;
          }
        }
        exploration->visualizeClustersConsole();
      }

      global_iterattions++;
      return false;              
    }

    /*
     * If received goal is not empty (x=0 y=0), drive the robot to this point
     * and mark that goal as seen in the last_goal_position vector!!!
     * Otherwise turn the robot by 90° to the right and search again for a better
     * frontier.
     */
    bool navigate(std::vector<double> goal) {
      ROS_INFO("nav");
      bool completed_navigation = false;
      if (goal_determined == true) {
        visualize_goal_point(goal.at(0), goal.at(1));
        counter++;
        ROS_INFO("GOAL %d:  x: %f      y: %f", counter, goal.at(0), goal.at(1));
        completed_navigation = move_robot(counter, goal.at(0), goal.at(1));
        if (!completed_navigation) {
          ROS_WARN("navigation failed:");
        }
        rotation_counter = 0;
      }
      else {
        rotation_counter++;
        ROS_INFO("In navigation .... cluster_available: %lu     counter: %d", exploration->clusters.size(), counter_waiting_for_clusters);			
        // if(clusters_available_in_pool.size() <= 0 || counter_waiting_for_clusters > 10) //(rotation_counter >= 2)
        if(exploration->clusters.size() == 0 || counter_waiting_for_clusters > 10) //(rotation_counter >= 2)
        {
          ROS_INFO("Iterating over GLOBAL COSTMAP to find a goal!!!!");
          std::vector<double> global_goal;
          std::vector<std::string> robot_str;
          bool global_costmap_goal = iterate_global_costmap(&global_goal, &robot_str);

          if(global_costmap_goal == false)
          {
            counter++;
            ROS_INFO("GOAL %d: BACK TO HOME   x: %f    y: %f", counter, home_point_x, home_point_y);

            /*
             * If the robot should drive to the home position
             * exploration_has_finished() has to be uncommented. 
             * It creates a file which is the immediate trigger to 
             * shut down the ros_node. Therefore the navigation process
             * is canceled . 
             */
            exploration_has_finished();
            visualize_goal_point(home_point_x, home_point_y);
            completed_navigation = false;
            for(int i = 0; i< 5; i++)
            {
              if(completed_navigation == false)
              {
                completed_navigation = move_robot(counter, home_point_x, home_point_y); 
              }
              else
              {
                break;
              }                                       
            }

            exploration_finished = true;
          }else
          {
            counter++;
            ROS_INFO("GOAL %d:  x: %f      y: %f", counter, global_goal.at(0), global_goal.at(1));
            visualize_goal_point(global_goal.at(0), global_goal.at(1));
            completed_navigation = move_robot(counter, global_goal.at(0), global_goal.at(1));
            rotation_counter = 0;
            if(completed_navigation == true)
            {
              exploration->calculate_travel_path(exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).x_coordinate, exploration->visited_frontiers.at(exploration->visited_frontiers.size()-1).y_coordinate);

              ROS_WARN("Storing visited...");
              exploration->storeVisitedFrontier(global_goal.at(0),global_goal.at(1),global_goal.at(2),robot_str.at(0), global_goal.at(3)); 
            }   
            else
            {
              ROS_WARN("failed to navigate with: Storing unreachable frontier with id: ");
              exploration->storeUnreachableFrontier(global_goal.at(0),global_goal.at(1),global_goal.at(2),robot_str.at(0), global_goal.at(3));
            }                          
          }
        } else
        {
          counter++;
          ROS_INFO("GOAL %d:  rotation", counter);
          completed_navigation = turn_robot(counter);
        }
      }      
      return(completed_navigation);
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

    // what actually moves the robot through the action client
    // returns true if the action succeeded, false on abort, 
    bool move_robot(int seq, double position_x, double position_y) {

      ROS_INFO("nav");
      exploration->next_auction_position_x = position_x;
      exploration->next_auction_position_y = position_y;

      /*
       * Move the robot with the help of an action client. Goal positions are
       * transmitted to the robot and feedback is given about the actual
       * driving state of the robot.
       */
      if (!costmap2d_local->getRobotPose(robotPose)) {
        ROS_ERROR("Failed to get RobotPose");
      }

      actionlib::SimpleActionClient < move_base_msgs::MoveBaseAction
        > ac("move_base", true);

      while (!ac.waitForServer(ros::Duration(10.0)))
        ;

      move_base_msgs::MoveBaseGoal goal_msgs;

      goal_msgs.target_pose.header.seq = seq;	// increase the sequence number
      //		goal_msgs.target_pose.header.stamp = ros::Time::now();
      goal_msgs.target_pose.header.frame_id = move_base_frame; //"map";
      goal_msgs.target_pose.pose.position.x = position_x;// - robotPose.getOrigin().getX(); //goals[0].pose.position.x;
      goal_msgs.target_pose.pose.position.y = position_y;// - robotPose.getOrigin().getY();
      goal_msgs.target_pose.pose.position.z = 0;
      goal_msgs.target_pose.pose.orientation.x = 0; //sin(angle/2); // goals[0].pose.orientation.x;
      goal_msgs.target_pose.pose.orientation.y = 0;
      goal_msgs.target_pose.pose.orientation.z = 0;
      goal_msgs.target_pose.pose.orientation.w = 1;

      ac.sendGoal(goal_msgs);

      //ac.waitForResult(ros::Duration(20)); EDIT Peter: Test if it also works with smaller value!
      ac.waitForResult(ros::Duration(waitForResult)); //here Parameter!
      while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
      {
        ros::Duration(0.5).sleep();
      }
      ROS_INFO("Not longer PENDING");

      while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
      {
        ros::Duration(0.5).sleep();
      }
      ROS_INFO("Not longer ACTIVE");

      while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
          ROS_INFO("ABORTED");

          exploration->next_auction_position_x = robotPose.getOrigin().getX();
          exploration->next_auction_position_y = robotPose.getOrigin().getY();
          return false;
        }
      }
      ROS_INFO("TARGET REACHED");

      exploration->next_auction_position_x = robotPose.getOrigin().getX();
      exploration->next_auction_position_y = robotPose.getOrigin().getY();
      return true;
    }

    bool turn_robot(int seq) {

      double angle = 45;

      if (!costmap2d_local->getRobotPose(robotPose)) {
        ROS_ERROR("Failed to get RobotPose");
      }

      actionlib::SimpleActionClient < move_base_msgs::MoveBaseAction
        > ac("move_base", true);
      while (!ac.waitForServer(ros::Duration(10.0)))
        ;

      move_base_msgs::MoveBaseGoal goal_msgs;

      goal_msgs.target_pose.header.seq = seq;	// increase the sequence number
      goal_msgs.target_pose.header.stamp = ros::Time::now();

      goal_msgs.target_pose.header.frame_id = move_base_frame;//"map";
      goal_msgs.target_pose.pose.position.x = robotPose.getOrigin().getX();
      goal_msgs.target_pose.pose.position.y = robotPose.getOrigin().getY();
      goal_msgs.target_pose.pose.position.z = 0;
      goal_msgs.target_pose.pose.orientation.x = 0; //sin(angle/2); // goals[0].pose.orientation.x;
      goal_msgs.target_pose.pose.orientation.y = 0;
      goal_msgs.target_pose.pose.orientation.z = sin(angle / 2);
      goal_msgs.target_pose.pose.orientation.w = cos(angle / 2);

      ac.sendGoal(goal_msgs);
      ac.waitForResult(ros::Duration(waitForResult));

      while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
      {
        ros::Duration(0.5).sleep();
      }
      ROS_INFO("Not longer PENDING");

      while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
      {
        ros::Duration(0.5).sleep();
      }
      ROS_INFO("Not longer ACTIVE");

      while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
          ROS_INFO("ABORTED");                                
          return false;
        }
      }
      ROS_INFO("ROTATION ACCOMPLISHED");
      return true;
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

    void globalCostmapCallback(nav_msgs::OccupancyGridPtr msg) {
      nav_msgs::OccupancyGrid costmap = *msg.get();
      exploration->updateGlobalCostmap(&costmap);
    }

    bool target_reached(void) {
      ROS_INFO("nav");

      ros::NodeHandle nh_sub_move_base;
      sub_move_base = nh_sub_move_base.subscribe("feedback", 1000,
          &Explorer::feedbackCallback, this);

      while (feedback_value != feedback_succeed_value)
        ;

      return true;
    }

  public:

    struct map_progress_t
    {
      double local_freespace;
      double global_freespace;
      double time;
    } map_progress;

    ros::Subscriber sub_move_base, sub_obstacle;    

    // create a costmap
    costmap_2d::Costmap2DROS* costmap2d_local;
    costmap_2d::Costmap2DROS* costmap2d_local_size;
    costmap_2d::Costmap2DROS* costmap2d_global;
    costmap_2d::Costmap2D costmap;

    std::vector<map_progress_t> map_progress_during_exploration;

    std::vector<int> clusters_available_in_pool;

    int home_position_x, home_position_y;
    int robot_id, number_of_robots;
    int frontier_selection, costmap_width, global_costmap_iteration, number_unreachable_frontiers_for_cluster;
    int counter_waiting_for_clusters;

    double robot_home_position_x, robot_home_position_y, costmap_resolution;
    bool Simulation, goal_determined;
    bool robot_prefix_empty;
    int accessing_cluster, cluster_element_size, cluster_element;
    int global_iterattions;
    bool cluster_flag, cluster_initialize_flag;
    int global_iterations_counter; 
    int waitForResult;
    std::string move_base_frame;
    std::string robot_prefix;               /// The prefix for the robot when used in simulation
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

    //Create a move_base_msgs to define a goal to steer the robot to
    move_base_msgs::MoveBaseActionGoal action_goal_msg;
    move_base_msgs::MoveBaseActionFeedback feedback_msgs;

    geometry_msgs::PointStamped goalPoint;
    geometry_msgs::PointStamped homePoint;

    std::vector<geometry_msgs::PoseStamped> goals;
    tf::Stamped<tf::Pose> robotPose;

    explorationPlanner::ExplorationPlanner *exploration;       

    double x_val, y_val, home_point_x, home_point_y;
    int seq, feedback_value, feedback_succeed_value, rotation_counter,
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
    costmap_mutex.lock(); 
    ros::spinOnce();
    costmap_mutex.unlock();

    ros::Duration(0.1).sleep();
  }

  thr_explore.interrupt();

  thr_explore.join();

  return 0;
}
