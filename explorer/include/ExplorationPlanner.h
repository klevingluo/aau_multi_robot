#ifndef PLANNER_H___
#define PLANNER_H___

#include "ExplorationPlanner.h"
#include <navfn/navfn_ros.h>
#include "ros/ros.h"
#include "hungarian.h"
#include <ros/console.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <adhoc_communication/ExpFrontier.h>
#include <adhoc_communication/ExpCluster.h>
#include <adhoc_communication/ExpAuction.h>
#include <adhoc_communication/MmPoint.h>
#include <adhoc_communication/MmListOfPoints.h>
#include <map_merger/TransformPoint.h>

namespace explorationPlanner
{
  class ExplorationPlanner
  {
    public:
      struct frontier_t
      {
        int id;
        int detected_by_robot;  
        std::string detected_by_robot_str; /** robot this was detected by */
        int mapx;
        int mapy;
        int map_index;
        double robot_home_x; 
        double robot_home_y;
        double x_coordinate;
        double y_coordinate;
        int distance_to_robot;
        int dist_to_robot;
      } frontier, unreachable_frontier; 

      struct responded_t
      {
        int robot_number; 
        std::string robot_str;
        int auction_number; 
      };

      struct cluster_t
      {
        std::vector<frontier_t> cluster_element;
        int id;
        int unreachable_frontier_count;
      } cluster;

      struct transform_point_t
      {
        int id; 
        std::string robot_str;
      };

      struct requested_cluster_t
      {
        std::vector<transform_point_t> requested_ids;
        int own_cluster_id; 
      };

      struct auction_pair_t
      {
        int bid_value;
        int cluster_id;
      };

      struct auction_element_t
      {
        int robot_id; 
        std::string detected_by_robot_str;
        std::vector<auction_pair_t> auction_element;
      };



      struct compare_pair_t
      {
        int cluster_id;
        int identical_ids;
      };

      boost::mutex store_frontier_mutex, store_visited_mutex, store_negotiation_mutex;
      boost::mutex publish_subscribe_mutex, callback_mutex, negotiation_mutex, negotiation_callback_mutex; 
      boost::mutex position_mutex, auction_mutex;
      boost::mutex cluster_mutex;
      boost::thread thr_auction_status, thr_auction_pid;

      ros::ServiceClient client;

      ros::Publisher pub_frontiers, pub_visited_frontiers, pub_negotion, pub_negotion_first, pub_Point;
      ros::Publisher pub_frontiers_points; 
      ros::Publisher pub_auctioning, pub_auctioning_status; 
      ros::Subscriber sub_frontiers, sub_visited_frontiers, sub_negotioation, sub_negotioation_first;
      ros::Subscriber sub_control;
      ros::Subscriber sub_auctioning, sub_auctioning_status;
      ros::Subscriber sub_position, sub_robot;        

      ros::Publisher pub_auctioning_first; 
      ros::Subscriber sub_auctioning_first; 

      ros::Publisher pub_clusters;

      ros::NodeHandle nh_frontier, nh_visited_frontier;
      ros::NodeHandle nh_Point, nh_visited_Point, nh_frontiers_points;
      ros::NodeHandle nh_transform, nh_negotiation, nh_negotiation_first;
      ros::NodeHandle nh_auction, nh_auction_status; 
      ros::NodeHandle nh_cluster, nh_cluster_grid;
      ros::NodeHandle nh_position, nh_robot;
      ros::NodeHandle nh_control; 
      ros::NodeHandle *nh_service;
      ros::NodeHandle nh; 
      ros::NodeHandle nh_auction_first;

      ros::MultiThreadedSpinner spinner;

      std::vector<int> allFrontiers;

      std::vector<auction_element_t> auction;
      std::vector<frontier_t> frontiers;
      std::vector<frontier_t> visited_frontiers;
      std::vector<frontier_t> unreachable_frontiers;

      std::vector<frontier_t> seen_frontier_list;
      std::vector<frontier_t> negotiation_list, my_negotiation_list;

      std::vector <responded_t> robots_already_responded;

      std::vector <double> goal_buffer_x;
      std::vector <double> goal_buffer_y;

      std::vector<double> last_goal_position_x;
      std::vector<double> last_goal_position_y;

      std::vector<cluster_t> clusters;
      std::vector<adhoc_communication::ExpCluster> unrecognized_occupied_clusters;

      std::vector<transform_point_t> transformedPointsFromOtherRobot_frontiers, transformedPointsFromOtherRobot_visited_frontiers;

      std::vector<int> already_used_ids, id_previously_detected;

      adhoc_communication::MmListOfPoints other_robots_positions; 

      tf::TransformListener listener;
      tf::StampedTransform transform;
      geometry_msgs::Point robotPose;

      map_merger::TransformPoint service_message;

      navfn::NavfnROS nav;

      ros::ServiceClient ssendFrontier, ssendAuction;

      std::string trajectory_strategy;
      bool first_run, first_negotiation_run;

      int number_of_auction_runs;
      int cluster_id, cluster_cells_seq_number;
      int number_of_completed_auctions, number_of_uncompleted_auctions;
      bool initialized_planner;
      bool auction_start, auction_finished;
      bool robot_prefix_empty_param;
      int number_of_auction_bids_received, number_of_robots;
      double other_robots_position_x, other_robots_position_y;
      int auction_id_number; 
      int auction_pid;
      int frontier_seq_number;
      double exploration_travel_path_global;
      double x_value,y_value,w_value,x_distance,y_distance,robot_pose_x,robot_pose_y;
      int goal_buffer_counter,goal_buffer_length,random_number, random_value;
      int inflated,free,lethal,unknown;
      double free_space;
      int frontier_point_message;
      int frontier_id_count, visited_frontier_id_count,unreachable_frontier_id_count;
      int goal_point_simulated_message, start_point_simulated_message;
      bool simulation;
      int robot_name;
      double robot_home_x, robot_home_y;
      std::string move_base_frame, robots_in_simulation;
      std::string robot_str;
      std::vector<std::string> new_robots;
      int next_auction_position_x, next_auction_position_y;

      ExplorationPlanner(int robot_id, bool robot_prefix_empty, std::string robot_name_parameter);
      bool respondToAuction(std::vector<requested_cluster_t> requested_cluster_ids, int auction_id_number);
      bool distanceToRobot(frontier_t front1, frontier_t front2);
      bool clusterIdToElementIds(int cluster_id, std::vector<transform_point_t>* occupied_ids);
      bool initialize_auctioning(std::vector<double> *final_goal);
      bool auctioning(std::vector<double> *final_goal, std::vector<int> *clusters_available_in_pool, std::vector<std::string> *robot_str_name);
      bool selectClusterBasedOnAuction(std::vector<double> *goal, std::vector<int> *cluster_in_use_already_count, std::vector<std::string> *robot_str_name_to_return);
      bool InitSelectClusterBasedOnAuction(std::vector<double> *goal);
      int calculateAuctionBID(int cluster_number, std::string strategy, int robot_x, int robot_y);
      std::string lookupRobotName(int robot_name_int);
      //            void auctionStatusCallback(const adhoc_communication::AuctionStatus::ConstPtr& msg);
      //            void controlCallback(const bla& msg);
      int calculate_travel_path(double x, double y);
      void updateLocalCostmap(nav_msgs::OccupancyGrid*);
      void updateGlobalCostmap(nav_msgs::OccupancyGrid*);
      void updateOdom(nav_msgs::Odometry*);
      int estimate_trajectory_plan(double start_x, double start_y, double target_x, double target_y);
      void Callbacks();
      //            void new_robot_callback(const std_msgs::StringConstPtr &msg);
      int checkClustersID(adhoc_communication::ExpCluster cluster_to_check);
      bool determine_goal(int strategy, std::vector<double> *final_goal, int count, int actual_cluster_id, std::vector<std::string> *robot_str_name);
      void sort(int strategy);
      void simulate();
      void visualize_Frontiers();
      void visualize_visited_Frontiers();
      void visualizeClustersConsole();
      void findFrontiers();
      void clearUnreachableFrontiers();
      bool storeFrontier(int x, int y, int map_index, int detected_by_robot, std::string detected_by_robot_str, int id);
      bool storeVisitedFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id);
      bool storeUnreachableFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id);
      bool removeStoredFrontier(int id, std::string detected_by_robot_str);
      bool publish_negotiation_list(frontier_t negotiation_frontier, int cluster);
      bool subscribe_negotiation_list();
      bool subscribe_frontier_list();
      bool subscribe_visited_frontier_list();
      void negotiationCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg);
      void visited_frontierCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg);
      void frontierCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg);
      void auctionCallback(const adhoc_communication::ExpAuction::ConstPtr& msg);
      void positionCallback(const adhoc_communication::MmListOfPoints::ConstPtr& msg);
      void setRobotConfig(int name, double robot_home_position_x, double robot_home_position_y, std::string frame);
      bool check_trajectory_plan();
      int check_trajectory_plan(double x, double y);
      bool negotiate_Frontier(double x, double y, int detected_by, int id, int cluster);
      bool clusterFrontiers();

      bool transformToOwnCoordinates_frontiers();

      bool sendToMulticast(std::string multi_cast_group, adhoc_communication::ExpFrontier frontier_to_send, std::string topic);
      bool sendToMulticastAuction(std::string multi_cast_group, adhoc_communication::ExpAuction auction_to_send, std::string topic);

    private:
      bool auction_running;

      void home_position_(const geometry_msgs::PointStamped::ConstPtr& msg);
      void clearFrontiers();
      int isFrontier(int point);
      bool isFree(int point);
      inline bool isValid(int point);
      inline void getAdjacentPoints(int point, int points[]);
      int backoff (int point);

      nav_msgs::OccupancyGrid *costmap_ros_;
      nav_msgs::OccupancyGrid *costmap_global_ros_;
      visualization_msgs::Marker createMarker();

      ros::Publisher visualization_pub_;

      std::vector<signed char> occupancy_grid_array_;
      unsigned int* exploration_trans_array_;
      unsigned int* obstacle_trans_array_;
      int* frontier_map_array_;
      int home_position_x_,home_position_y_;

      std::string name;
      unsigned int map_width_;
      unsigned int map_height_;
      unsigned int num_map_cells_;
      geometry_msgs::Pose map_origin_;
      nav_msgs::Odometry *odom;
      float resolution_;
  };
}
#endif
