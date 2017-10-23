#include "ExplorationPlanner.h"
#include "boost_matrix.h"
#include "hungarian.h"
#include "munkres.h"
#include "ros/ros.h"
#include <adhoc_communication/ExpAuction.h>
#include <adhoc_communication/ExpCluster.h>
#include <adhoc_communication/ExpClusterElement.h>
#include <adhoc_communication/ExpFrontier.h>
#include <adhoc_communication/ExpFrontierElement.h>
#include <adhoc_communication/MmListOfPoints.h>
#include <adhoc_communication/SendExpAuction.h>
#include <adhoc_communication/SendExpCluster.h>
#include <adhoc_communication/SendExpFrontier.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PolygonStamped.h>
#include <map_merger/TransformPoint.h>
#include <math.h>
#include <nav_msgs/GridCells.h>
#include <navfn/navfn_ros.h>
#include <ros/console.h>
#include <stdlib.h>
#include <string.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define MAX_DISTANCE 2000			// max distance to starting point
#define MAX_GOAL_RANGE 0.2			// min distance between frontiers (search)
#define MINIMAL_FRONTIER_RANGE 0.2	// distance between frontiers (selection)
#define INNER_DISTANCE 5			// radius (in cells) around goal point without obstacles (backoff goal point)
#define MAX_NEIGHBOR_DIST 1			// radius (in cells) around selected goal without obstacles
#define CLUSTER_MERGING_DIST 0.8	// max (euclidean) distance between clusters that are merged

using namespace explorationPlanner;

template <typename T>
std::string NumberToString ( T Number ) {
  std::ostringstream ss;
  ss << Number;
  return ss.str();
}

ExplorationPlanner::ExplorationPlanner(int robot_id, bool robot_prefix_empty, std::string robot_name_parameter):
  costmap_ros_(0), 
  occupancy_grid_array_(0), 
  exploration_trans_array_(0), 
  obstacle_trans_array_(0), 
  frontier_map_array_(0), 
  is_goal_array_(0), 
  map_width_(0), 
  map_height_(0), 
  num_map_cells_(0), 
  initialized_(false), 
  last_mode_(FRONTIER_EXPLORE), 
  p_alpha_(0), 
  p_dist_for_goal_reached_(1), 
  p_goal_angle_penalty_(0), 
  p_min_frontier_size_(0), 
  p_min_obstacle_dist_(0), 
  p_plan_in_unknown_( true), 
  p_same_frontier_dist_(0), 
  p_use_inflated_obs_(false), 
  previous_goal_( 0), 
  inflated(0), 
  lethal(0), 
  free(0), 
  threshold_free(127) , 
  threshold_inflated(252), 
  threshold_lethal(253),
  frontier_id_count(0), 
  exploration_travel_path_global(0), 
  cluster_id(0), 
  initialized_planner(false), 
  auction_is_running(false), 
  auction_start(false), 
  auction_finished(true), 
  auction_id_number(1), 
  next_auction_position_x(0), 
  next_auction_position_y(0), 
  other_robots_position_x(0), 
  other_robots_position_y(0),
  number_of_completed_auctions(0), 
  number_of_uncompleted_auctions(0), 
  first_run(true),
  first_negotiation_run(true), 
  robot_prefix_empty_param(false)
{
  trajectory_strategy = "euclidean";
  robot_prefix_empty_param = robot_prefix_empty;

  responded_t init_responded;
  init_responded.auction_number = 0;
  init_responded.robot_number = 0;
  robots_already_responded.push_back(init_responded);

  auction_running = false;
  std::stringstream robot_number;
  robot_number << robot_id;

  std::string prefix = "/robot_";
  std::string robo_name = prefix.append(robot_number.str());   

  std::string sendFrontier_msgs = robo_name +"/adhoc_communication/send_frontier";
  std::string sendAuction_msgs  = robo_name +"/adhoc_communication/send_auction";

  ros::NodeHandle tmp;
  nh_service = &tmp;

  ROS_DEBUG("Sending frontier: '%s'     SendAuction: '%s'", sendFrontier_msgs.c_str(), sendAuction_msgs.c_str());

  ssendFrontier = nh_service->serviceClient<adhoc_communication::SendExpFrontier>(sendFrontier_msgs);
  ssendAuction = nh_service->serviceClient<adhoc_communication::SendExpAuction>(sendAuction_msgs);

  pub_visited_frontiers_points = nh_visited_Point.advertise <visualization_msgs::MarkerArray> ("visitedfrontierPoints", 2000, true);

  pub_clusters = nh.advertise <visualization_msgs::MarkerArray> ("clusters", 200, true);

  pub_Point = nh_Point.advertise < geometry_msgs::PointStamped> ("goalPoint", 100, true);
  pub_frontiers_points = nh_frontiers_points.advertise <visualization_msgs::MarkerArray> ("frontierPoints", 2000, true);

  sub_frontiers = nh_frontier.subscribe(robo_name+"/frontiers", 
      10000, 
      &ExplorationPlanner::frontierCallback, 
      this);

  sub_visited_frontiers = nh_visited_frontier.subscribe(
      robo_name+"/visited_frontiers", 
      10000, 
      &ExplorationPlanner::visited_frontierCallback, 
      this);

  sub_negotioation = nh_negotiation.subscribe(
      robo_name+"/negotiation_list", 
      10000, 
      &ExplorationPlanner::negotiationCallback, 
      this); 

  sub_auctioning = nh_auction.subscribe(
      robo_name+"/auction", 
      1000, 
      &ExplorationPlanner::auctionCallback, 
      this); 

  sub_position = nh_position.subscribe(robo_name+"/all_positions", 
      1000, 
      &ExplorationPlanner::positionCallback, 
      this);

  srand((unsigned)time(0));
}

void ExplorationPlanner::initialize_planner(std::string name,
    costmap_2d::Costmap2DROS *costmap, costmap_2d::Costmap2DROS *costmap_global) {

  ROS_INFO("Initializing the planner");

  this->costmap_ros_ = costmap;
  this->costmap_global_ros_ = costmap_global;

  if(initialized_planner == false) {
    nav.initialize("navigation_path", costmap_global_ros_);
    initialized_planner = true;
  }
  //Occupancy_grid_array is updated here
  this->setupMapData();

  last_mode_ = FRONTIER_EXPLORE;
  this->initialized_ = true;

  /*
   * reset all counter variables, used to count the number of according blocks
   * within the occupancy grid.
   */
  unknown = 0, free = 0, lethal = 0, inflated = 0;
}

/**
 * write frontiers from list to clusters
 */
bool ExplorationPlanner::clusterFrontiers() {

  bool cluster_found_flag = false, same_id = false;

  for(int i = 0; i < frontiers.size(); i++)
  {
    ROS_DEBUG("Frontier at: %d   and cluster size: %lu",i, clusters.size());
    cluster_found_flag = false;
    bool frontier_used = false;
    same_id = false;

    for(int j = 0; j < clusters.size(); j++)
    {
      ROS_DEBUG("cluster %d contains %lu elements", j, clusters.at(j).cluster_element.size());
      for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
      {
        ROS_DEBUG("accessing cluster %d  and element: %d", j, n);

        if(fabs(frontiers.at(i).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate) < MAX_NEIGHBOR_DIST && fabs(frontiers.at(i).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate) < MAX_NEIGHBOR_DIST) 
        {    
          for(int m = 0; m < clusters.at(j).cluster_element.size(); m++)
          {    
            ROS_DEBUG("checking id %d with element id: %d",frontiers.at(i).id,clusters.at(j).cluster_element.at(m).id);

            if(frontiers.at(i).id == clusters.at(j).cluster_element.at(m).id)
            {
              frontier_used = true;
              same_id = true;
              break;
            } 


          }  
          if(same_id == false)
          {
            cluster_found_flag = true; 
          }                 
        }
        if(same_id == true || cluster_found_flag == true)
        {
          break;
        }
      } 
      if(same_id == true) {
        break;
      } else {
        if(cluster_found_flag == true) {                    
          ROS_DEBUG("Frontier: %d attached", frontiers.at(i).id);
          clusters.at(j).cluster_element.push_back(frontiers.at(i));  
          frontier_used = true;
          break;
        }    
      }
    }
    if(cluster_found_flag == false && same_id == false) {
      ROS_DEBUG("ADD CLUSTER");
      cluster_t cluster_new;
      cluster_new.cluster_element.push_back(frontiers.at(i));
      cluster_new.id = (robot_name * 10000) + cluster_id++;

      cluster_mutex.lock();
      clusters.push_back(cluster_new); 
      cluster_mutex.unlock();

      ROS_DEBUG("Frontier: %d in new cluster", frontiers.at(i).id);
      frontier_used = true;
    }   
    if(frontier_used == false)
    {
      ROS_WARN("Frontier: %d not used", frontiers.at(i).id);
    }
  }  
}

/**
 * outputs clusters to the rviz console
 */
void ExplorationPlanner::visualizeClustersConsole() {

  // action 3: delete all markers
  visualization_msgs::Marker deleteall;
  deleteall.action = 3;

  visualization_msgs::MarkerArray deleteallArray;
  deleteallArray.markers.push_back(deleteall);              

  pub_clusters.publish <visualization_msgs::MarkerArray>(deleteallArray);

  visualization_msgs::MarkerArray clustersArray;
  for(int j = 0; j < clusters.size(); j++) {

    double color_r = (double)rand() / RAND_MAX;
    double color_g = (double)rand() / RAND_MAX;    
    double color_b = (double)rand() / RAND_MAX;

    for(int n = 0; n < clusters.at(j).cluster_element.size(); n++) {
      visualization_msgs::Marker marker;

      marker.header.frame_id = move_base_frame;    
      marker.header.stamp = ros::Time::now();
      marker.header.seq = frontier_seq_number++;
      marker.ns = "clusters";
      marker.id = frontier_seq_number;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = clusters.at(j).cluster_element.at(n).x_coordinate;
      marker.pose.position.y = clusters.at(j).cluster_element.at(n).y_coordinate;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;

      marker.color.a = 1.0;
      marker.color.r = color_r;
      marker.color.g = color_g;
      marker.color.b = color_b;               

      clustersArray.markers.push_back(marker);              
    }           
  }
  pub_clusters.publish <visualization_msgs::MarkerArray>(clustersArray);
}

bool ExplorationPlanner::transformToOwnCoordinates(int x, int y, int detected_by_robot, int &newx, int &newy)
{
  std::string robo_name = "robot_" + detected_by_robot;
  std::string robo_name2 = "robot_" + robot_name;

  std::string service_topic = robo_name2.append("/map_merger/transformPoint"); // FIXME for real scenario!!! robot_might not be used here

  client = nh_transform.serviceClient<map_merger::TransformPoint>(service_topic);

  service_message.request.point.x = x;
  service_message.request.point.y = y;
  service_message.request.point.src_robot = robo_name;

  if(client.call(service_message))
  {
    newx = service_message.response.point.x; 
    newy = service_message.response.point.y;
    return true;
  }
  return false;
}

// writes all points from fontiers to
// transformedPointsFromOtherRobot_frontiers
bool ExplorationPlanner::transformToOwnCoordinates_frontiers() {
  ROS_INFO("Transform frontier coordinates");

  store_frontier_mutex.lock();

  for(int i = 0; i < frontiers.size(); i++) {        
    bool same_robot = false;
    if(frontiers.at(i).detected_by_robot != robot_name) {
      bool transform_flag = false;
      for(int j=0; j < transformedPointsFromOtherRobot_frontiers.size(); j++)
      {
        if(transformedPointsFromOtherRobot_frontiers.at(j).id == frontiers.at(i).id)
        {
          transform_flag = true;
          break;
        }
      }

      if(transform_flag != true)
      {
        int newx, newy;
        transformToOwnCoordinates(
            frontiers.at(i).x_coordinate, 
            frontiers.at(i).y_coordinate, 
            frontiers.at(i).detected_by_robot, 
            newx, 
            newy);

        frontiers.at(i).x_coordinate = newx; 
        frontiers.at(i).y_coordinate = newy;

        transform_point_t transform_point;
        transform_point.id = frontiers.at(i).id;

        transformedPointsFromOtherRobot_frontiers.push_back(transform_point);
      }
    }
  }
  store_frontier_mutex.unlock();
  ROS_INFO(" Transform frontier coordinates DONE");
}

bool ExplorationPlanner::transformToOwnCoordinates_visited_frontiers() {
  ROS_INFO("Transform Visited Frontier Coordinates");

  for(int i = 0; i < visited_frontiers.size(); i++) {
    bool same_robot = false;
    if(visited_frontiers.at(i).detected_by_robot != robot_name) {
      bool transform_flag = false;
      for(int j=0; j < transformedPointsFromOtherRobot_visited_frontiers.size(); j++) {
        if(transformedPointsFromOtherRobot_visited_frontiers.at(j).id == visited_frontiers.at(i).id && visited_frontiers.at(i).detected_by_robot_str.compare(transformedPointsFromOtherRobot_visited_frontiers.at(j).robot_str)== 0) {
          transform_flag = true;
          break;
        }
      }

      if(transform_flag != true) {
        int newx, newy;
        transformToOwnCoordinates(
            frontiers.at(i).x_coordinate, 
            frontiers.at(i).y_coordinate, 
            frontiers.at(i).detected_by_robot, 
            newx, 
            newy);

        visited_frontiers.at(i).x_coordinate = newx; 
        visited_frontiers.at(i).y_coordinate = newy;

        transform_point_t transform_point;
        transform_point.id = visited_frontiers.at(i).id;

        transformedPointsFromOtherRobot_visited_frontiers.push_back(transform_point);
        ROS_DEBUG("New visited x: %.1f   y: %.1f",service_message.response.point.x, service_message.response.point.y);                   
      }
    }
  }
  ROS_INFO(" Transform visited frontier coordinates DONE");
}

/**
 * is the plan successful?, write the distances to
 frontiers.at(i).distance_to_robot = distance;
 */
bool ExplorationPlanner::check_trajectory_plan() {       
  for(int i = 0; i<10; i++) {
    if(frontiers.size() > i) {   
      int distance = check_trajectory_plan(
          frontiers.at(i).x_coordinate, 
          frontiers.at(i).y_coordinate);

      frontiers.at(i).distance_to_robot = distance;
      ROS_DEBUG("Distance: %d Frontier: %d",distance, frontiers.at(i).id);
    }else
      break;
  }
  ROS_DEBUG("trajectory finished");
  return(true);
}


/**
 * same but adds the path length to
 exploration_travel_path_global = exploration_travel_path_global + distance;  
 * returns 0 on failure though
 */
int ExplorationPlanner::calculate_travel_path(double x, double y) {
  int distance = check_trajectory_plan(x,y);

  ROS_INFO("Plan elements: %f", (double)distance*0.02);

  if(distance != -1) {
    exploration_travel_path_global = exploration_travel_path_global + distance;  
    return distance;
  }else
  {
    return 0;
  }
}

/**
 * returns the distance to x, y, or neg1 if error
 */
int ExplorationPlanner::check_trajectory_plan(double x, double y) {       
  return estimate_trajectory_plan (
      robotPose.getOrigin().getX(),
      robotPose.getOrigin().getY(),
      x,
      y);
}

int ExplorationPlanner::estimate_trajectory_plan(
    double start_x, 
    double start_y, 
    double target_x, 
    double target_y) 
{       
  geometry_msgs::PoseStamped goalPointSimulated, startPointSimulated;
  int distance; 

  ROS_DEBUG("Check Trajectory");
  if (!costmap_ros_->getRobotPose(robotPose))
  {       
    ROS_ERROR("Failed to get RobotPose");
  }

  startPointSimulated.header.seq = start_point_simulated_message++;	// increase the sequence number
  startPointSimulated.header.stamp = ros::Time::now();
  startPointSimulated.header.frame_id = move_base_frame;
  startPointSimulated.pose.position.x = start_x;
  startPointSimulated.pose.position.y = start_y;
  startPointSimulated.pose.position.z = 0;
  startPointSimulated.pose.orientation.x = 0;
  startPointSimulated.pose.orientation.y = 0;
  startPointSimulated.pose.orientation.z = 0;
  startPointSimulated.pose.orientation.w = 1;

  goalPointSimulated.header.seq = goal_point_simulated_message++;	// increase the sequence number
  goalPointSimulated.header.stamp = ros::Time::now();
  goalPointSimulated.header.frame_id = move_base_frame;

  goalPointSimulated.pose.position.x = target_x;
  goalPointSimulated.pose.position.y = target_y;

  goalPointSimulated.pose.position.z = 0;
  goalPointSimulated.pose.orientation.x = 0;
  goalPointSimulated.pose.orientation.y = 0;
  goalPointSimulated.pose.orientation.z = 0;
  goalPointSimulated.pose.orientation.w = 1;

  std::vector<geometry_msgs::PoseStamped> global_plan;

  bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);

  if(successful == true)
  {
    distance =  global_plan.size();      
    return distance;
  }else
  {
    return -1;
  }
}

void ExplorationPlanner::setRobotConfig(int name, double robot_home_position_x, double robot_home_position_y, std::string frame)
{
  robot_name = name;
  robot_home_x = robot_home_position_x;
  robot_home_y = robot_home_position_y;
  move_base_frame = frame;
}

void ExplorationPlanner::home_position_(const geometry_msgs::PointStamped::ConstPtr& msg) {
  home_position_x_ = msg.get()->point.x;
  home_position_y_ = msg.get()->point.y;
}

/**
 * adds a frontier to the frontiers list
 */
bool ExplorationPlanner::storeFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id) {
  frontier_t new_frontier;

  if(detected_by_robot != robot_name)
  {
    new_frontier.id = id;
  }
  else
  {
    new_frontier.id = (robot_name * 10000) + frontier_id_count++; 
  }

  new_frontier.detected_by_robot = detected_by_robot;
  new_frontier.x_coordinate = x;
  new_frontier.y_coordinate = y;

  store_frontier_mutex.lock(); 
  frontiers.push_back(new_frontier);
  store_frontier_mutex.unlock();

  return true;
}

bool ExplorationPlanner::removeStoredFrontier(int id, std::string detected_by_robot_str) {
  for(int i= 0; i < frontiers.size(); i++)
  {
    if(frontiers.at(i).id == id)
    {
      store_frontier_mutex.lock();
      frontiers.erase(frontiers.begin()+i);
      if(i > 0)
      {
        i --;
      }
      store_frontier_mutex.unlock();
      break;
    }
  }
  return true;
}

bool ExplorationPlanner::storeVisitedFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id)
{
  frontier_t visited_frontier;

  if(detected_by_robot != robot_name)
  {
    visited_frontier.id = id;
  }
  else
  {
    visited_frontier.id = (robot_name * 10000) + visited_frontier_id_count++; 
  }

  visited_frontier.detected_by_robot = detected_by_robot;
  visited_frontier.x_coordinate = x;
  visited_frontier.y_coordinate = y;

  store_visited_mutex.lock();
  visited_frontiers.push_back(visited_frontier);
  store_visited_mutex.unlock();

  bool break_flag = false; 
  for(int i = 0; i < clusters.size(); i++)
  {
    for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
    {
      if(clusters.at(i).cluster_element.at(j).id == id)
      {
        ROS_DEBUG("Set cluster unreachable count to 0");
        clusters.at(i).unreachable_frontier_count = 0;
        break_flag = true; 
        break;
      }           
    }
    if(break_flag == true)
    {
      break;
    }
  }

  return true;
}

/**
 * store a new frontier, mark it as unreachable, 
 * then check it it is in any existing clusters, add to unreachable count if so
 */
bool ExplorationPlanner::storeUnreachableFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id)
{
  frontier_t unreachable_frontier;

  if(detected_by_robot != robot_name)
  {
    unreachable_frontier.id = id;
  }
  else
  {
    unreachable_frontier.id = (robot_name * 10000) + unreachable_frontier_id_count++; 
  }

  unreachable_frontier.detected_by_robot = detected_by_robot;
  unreachable_frontier.x_coordinate = x;
  unreachable_frontier.y_coordinate = y;

  frontiers.push_back(unreachable_frontier);


  bool break_flag = false; 
  for(int i = 0; i < clusters.size(); i++)
  {
    for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
    {
      if(clusters.at(i).cluster_element.at(j).id == id)
      {
        clusters.at(i).unreachable_frontier_count++;
        break_flag = true; 
        break;
      }           
    }
    if(break_flag == true)
    {
      ROS_INFO("unreachable frontier marked in cluster: %i", clusters.at(i).unreachable_frontier_count);
      break;
    }
  }

  return true;

}

bool ExplorationPlanner::publish_negotiation_list(
    frontier_t negotiation_frontier, 
    int cluster_number)
{
  adhoc_communication::ExpFrontier negotiation_msg;

  if(cluster_number != -1)
  {
    int cluster_vector_position = 0;
    for (int i = 0; i < clusters.size(); i++)
    {
      if(clusters.at(i).id == cluster_number)
      {
        ROS_DEBUG("Cluster ID: %d is at vector position: %d", cluster_number, i);
        cluster_vector_position = i;
        break;
      }
    }

    for(int i = 0; i < clusters.at(cluster_vector_position).cluster_element.size(); i++)
    {
      adhoc_communication::ExpFrontierElement negotiation_element;
      negotiation_element.x_coordinate = clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate;
      negotiation_element.y_coordinate = clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate;
      negotiation_element.detected_by_robot = clusters.at(cluster_vector_position).cluster_element.at(i).detected_by_robot;
      negotiation_element.id = clusters.at(cluster_vector_position).cluster_element.at(i).id;

      frontier_t new_frontier;
      new_frontier.x_coordinate = clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate;
      new_frontier.y_coordinate = clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate;
      new_frontier.detected_by_robot = clusters.at(cluster_vector_position).cluster_element.at(i).detected_by_robot;
      new_frontier.id = clusters.at(cluster_vector_position).cluster_element.at(i).id;

      negotiation_msg.frontier_element.push_back(negotiation_element);
      my_negotiation_list.push_back(new_frontier);
    }
    //        return true;
  }else
  {
    adhoc_communication::ExpFrontierElement negotiation_element;
    negotiation_element.detected_by_robot = negotiation_frontier.detected_by_robot;
    negotiation_element.x_coordinate = negotiation_frontier.x_coordinate;
    negotiation_element.y_coordinate = negotiation_frontier.y_coordinate;
    negotiation_element.id = negotiation_frontier.id;

    negotiation_msg.frontier_element.push_back(negotiation_element); //FIXME
  }

  sendToMulticast("mc_",negotiation_msg, "negotiation_list");

  first_negotiation_run = false; 
}

bool ExplorationPlanner::sendToMulticast(
    std::string multi_cast_group, 
    adhoc_communication::ExpFrontier frontier_to_send, 
    std::string topic)
{
  adhoc_communication::SendExpFrontier service_frontier; // create request of type any+

  std::stringstream robot_number;
  robot_number << robot_name;
  std::string prefix = "robot_";
  std::string robo_name = prefix.append(robot_number.str());   

  std::string destination_name = multi_cast_group + robo_name; //for multicast
  //    std::string destination_name = robo_name; // unicast

  ROS_INFO("sending to multicast group '%s' on topic: '%s'",destination_name.c_str(), topic.c_str());
  service_frontier.request.dst_robot = destination_name; 
  service_frontier.request.frontier = frontier_to_send;
  service_frontier.request.topic = topic;

  if (ssendFrontier.call(service_frontier))
  {
    ROS_DEBUG("Successfully called service  sendToMulticast");

    if(service_frontier.response.status)
    {
      ROS_DEBUG("adhoc comm returned successful transmission");
      return true;
    }
    else
    {
      ROS_DEBUG("Failed to send to multicast group %s!",destination_name.c_str());
      return false;
    }                  
  }
  else
  {
    ROS_WARN("Failed to call service sendToMulticast [%s]",ssendFrontier.getService().c_str());
    return false;
  }
}

bool ExplorationPlanner::sendToMulticastAuction(
    std::string multi_cast_group, 
    adhoc_communication::ExpAuction auction_to_send, 
    std::string topic)
{
  adhoc_communication::SendExpAuction service_auction; // create request of type any+

  std::stringstream robot_number;
  robot_number << robot_name;
  std::string prefix = "robot_";
  std::string robo_name = prefix.append(robot_number.str());    


  std::string destination_name = multi_cast_group + robo_name; //for multicast

  ROS_INFO("sending auction to multicast group '%s' on topic '%s'",destination_name.c_str(), topic.c_str());
  service_auction.request.dst_robot = destination_name; 
  service_auction.request.auction = auction_to_send;
  service_auction.request.topic = topic;

  if (ssendAuction.call(service_auction))
  {
    ROS_DEBUG("Successfully called service sendToMulticast");

    if(service_auction.response.status)
    {
      ROS_DEBUG("Auction was multicasted successfully.");
      return true;
    }
    else
    {
      ROS_WARN("Failed to send auction to mutlicast group %s!",destination_name.c_str());
      return false;
    }                  
  }
  else
  {
    ROS_WARN("Failed to call service sendToMulticastAuction [%s]",ssendAuction.getService().c_str());
    return false;
  }
}

void ExplorationPlanner::negotiationCallback(
    const adhoc_communication::ExpFrontier::ConstPtr& msg)
{  
  bool entry_found = false;

  adhoc_communication::ExpFrontierElement frontier_element; 
  for(int j = 0; j < msg.get()->frontier_element.size(); j++)
  {
    frontier_element = msg.get()->frontier_element.at(j);
    for(int i = 0; i < negotiation_list.size(); i++)
    {
      if(negotiation_list.at(i).id == frontier_element.id)
      {
        entry_found = true;       
      }  

    }
    if(entry_found == false)
    {
      ROS_DEBUG("Negotiation frontier with ID: %ld", frontier_element.id);
      frontier_t negotiation_frontier;
      negotiation_frontier.detected_by_robot = frontier_element.detected_by_robot;
      negotiation_frontier.id = frontier_element.id;
      negotiation_frontier.x_coordinate = frontier_element.x_coordinate;
      negotiation_frontier.y_coordinate = frontier_element.y_coordinate;

      store_negotiation_mutex.lock();
      negotiation_list.push_back(negotiation_frontier); 
      store_negotiation_mutex.unlock();
    }
  }
}

bool ExplorationPlanner::respondToAuction(
    std::vector<requested_cluster_t> requested_cluster_ids, 
    int auction_id_number) 
{
  adhoc_communication::ExpAuction auction_msgs;

  for(int n = 0; n < requested_cluster_ids.size(); n++) {
    cluster_mutex.lock();
    ROS_INFO("Responding to cluster ids: %d", requested_cluster_ids.at(n).own_cluster_id);
    for(int i = 0; i < clusters.size(); i++)
    {
      if(clusters.at(i).id == requested_cluster_ids.at(n).own_cluster_id)
      {
        //TODO:  figure out what exactly is happenning here
        adhoc_communication::ExpCluster cluster_msg;
        adhoc_communication::ExpClusterElement cluster_element_msg;
        for(int j = 0; j < requested_cluster_ids.at(n).requested_ids.size(); j++)
        {
          cluster_element_msg.id = requested_cluster_ids.at(n).requested_ids.at(j).id;
          cluster_msg.ids_contained.push_back(cluster_element_msg);
        }

        if (!costmap_global_ros_->getRobotPose(robotPose))
        {       
          ROS_ERROR("Failed to get RobotPose");
        }
        cluster_msg.bid = calculateAuctionBID(
            clusters.at(i).id, 
            trajectory_strategy,
            robotPose.getOrigin().getX(),
            robotPose.getOrigin().getY());
        auction_msgs.available_clusters.push_back(cluster_msg);
        break;
      }
    }
    cluster_mutex.unlock();
  }

  ROS_INFO("Robot %d publishes auction bids for all clusters", robot_name);

  auction_msgs.auction_status_message = false;
  auction_msgs.auction_id = auction_id_number; 

  std::stringstream ss;
  ss << robot_name; 
  std::string prefix = "";
  std::string robo_name = prefix.append(ss.str());  

  auction_msgs.robot_name = robo_name;

  sendToMulticastAuction("mc_", auction_msgs, "auction");
}

/**
 * calculates the bid for a cluster number, this is the distance to the cluster
 */
int ExplorationPlanner::calculateAuctionBID(
    int cluster_number, 
    std::string strategy,
    int robot_pose_x,
    int robot_pose_y)
{
  ROS_INFO("Calculating bid for: %d", cluster_number);
  if (!costmap_global_ros_->getRobotPose(robotPose))
  {       
    ROS_ERROR("Failed to get RobotPose");
  }
  int auction_bid = 0; 
  int cluster_vector_position = -1;
  bool cluster_could_be_found = false; 

  if(clusters.size() > 0)
  {                   
    for (int i = 0; i < clusters.size(); i++)
    {
      if(clusters.at(i).id == cluster_number)
      {
        cluster_vector_position = i; 
        cluster_could_be_found = true;
        break;
      }
    }                      
  }
  if(cluster_could_be_found == false)
  {     
    ROS_WARN("Cluster could not be found");
    return(-1); 
  }

  int distance = -1;
  for(int i = 0; i < clusters.at(cluster_vector_position).cluster_element.size(); i++)
  {

    double x = clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate - robot_pose_x;
    double y = clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate - robot_pose_y;        
    double euclidean_distance = x * x + y * y;

    if(strategy == "euclidean")
    {
      return euclidean_distance; 
    }else if(strategy == "trajectory")
    {
      distance = check_trajectory_plan(
          clusters.at(cluster_vector_position).cluster_element.at(i).x_coordinate, 
          clusters.at(cluster_vector_position).cluster_element.at(i).y_coordinate);

      if(distance > 0)
      {

        /*
         * Check if the distance calculation is plausible. 
         * The euclidean distance to this point need to be smaller then 
         * the simulated trajectory path. If this stattement is not valid
         * the trajectory calculation has failed. 
         */
        //            ROS_INFO("Euclidean distance: %f   trajectory_path: %f", sqrt(euclidean_distance), distance* costmap_ros_->getCostmap()->getResolution());
        if (distance * costmap_ros_->getCostmap()->getResolution() <= sqrt(euclidean_distance)*0.95) 
        {
          ROS_WARN("Euclidean distance smaller then trajectory distance to LOCAL CLUSTER!!!");
          //                return(-1);
        }else
        {
          return distance;
        }           
      }
    }
  }
  ROS_ERROR("Unable to calculate LOCAL BID at position %d  --> BID: %d", cluster_vector_position, distance);
  return(-1);
}

void ExplorationPlanner::positionCallback(const adhoc_communication::MmListOfPoints::ConstPtr& msg)
{
  ROS_DEBUG("Position Callback !!!!!");
  position_mutex.lock();

  other_robots_positions.positions.clear();
  ROS_INFO("positions size: %lu", msg.get()->positions.size());
  for(int i = 0; i < msg.get()->positions.size(); i++)
  {    
    other_robots_positions.positions.push_back(msg.get()->positions.at(i));
  }
  position_mutex.unlock();
}

/**
 * processes auction messages
 */
void ExplorationPlanner::auctionCallback(
    const adhoc_communication::ExpAuction::ConstPtr& msg)
{
  auction_running = true;
  int robots_int_name;

  robots_int_name = atoi(msg.get()->robot_name.c_str());
  if(robots_int_name != robot_name)
  {
    /*
     * TODO Check if following code is necessary or if simple navigation needs to 
     * periodically update the frontiers in the frontier thread.  
     */
    transformToOwnCoordinates_frontiers();
    transformToOwnCoordinates_visited_frontiers();

    clearVisitedFrontiers();                       
    clearUnreachableFrontiers();
    clearSeenFrontiers(costmap_global_ros_);       

    clearVisitedAndSeenFrontiersFromClusters();
    clusterFrontiers();

    if(msg.get()->auction_status_message == true)
    { 
      ROS_INFO("Calling Auction Status");

      auction_start = msg.get()->start_auction;
      auction_finished = msg.get()->auction_finished;
      adhoc_communication::ExpCluster occupied_ids;
      std::vector<requested_cluster_t> requested_cluster_ids;

      /*
       * Grep all occupied ids and try to convert them into clusters in own 
       * coordinate system. This ensures that all robots in the system know
       * which clusters had been occupied by others and do not select them
       * twice. 
       */
      if(msg.get()->occupied_ids.size() > 0 || msg.get()->requested_clusters.size() > 0)
      {
        for(int i = 0; i < msg.get()->occupied_ids.size(); i++)
        {
          adhoc_communication::ExpClusterElement cluster_element; 

          cluster_element.id = msg.get()->occupied_ids.at(i).id; 
          cluster_element.detected_by_robot_str = msg.get()->occupied_ids.at(i).detected_by_robot_str;

          occupied_ids.ids_contained.push_back(cluster_element);
        }
        int occupied_cluster_id = checkClustersID(occupied_ids);   
        ROS_INFO("Check occupied cluster to be the same. %d", occupied_cluster_id);

        if(occupied_cluster_id >=0)
        {
          ROS_INFO("Adding occupied cluster: %d", occupied_cluster_id);
          already_used_ids.push_back(occupied_cluster_id);
        }else
        {
          /* Store undetected Clusters in a struct for later processing */
          ROS_INFO("Adding occupied cluster as unrecognized!");
          unrecognized_occupied_clusters.push_back(occupied_ids);
        }
        /*
         * Now read from unrecognized cluster vector and try to convert
         * these ids to a valid cluster to know which had already been 
         * occupied.
         */
        for(int i = 0; i < unrecognized_occupied_clusters.size(); i++)
        {
          int unrecognized_occupied_cluster_id = checkClustersID(unrecognized_occupied_clusters.at(i));
          if(unrecognized_occupied_cluster_id >=0)
          {
            ROS_INFO("Unrecognized cluster: %d converted", unrecognized_occupied_cluster_id);
            already_used_ids.push_back(unrecognized_occupied_cluster_id);
            unrecognized_occupied_clusters.erase(unrecognized_occupied_clusters.begin() + i);

            if(i > 0)
              i--;
          }
        }


        /*
         * Grep the requested clusters to know which ones to answer to. 
         */
        for(int i = 0; i < msg.get()->requested_clusters.size(); i++)
        {    
          adhoc_communication::ExpCluster requested_cluster;
          requested_cluster = msg.get()->requested_clusters.at(i);

          int check_cluster_id = checkClustersID(requested_cluster);
          if(check_cluster_id >= 0)
          {
            requested_cluster_t new_cluster_request; 
            for(int j = 0; j < requested_cluster.ids_contained.size(); j++)
            {
              transform_point_t cluster_element_point;
              cluster_element_point.id = requested_cluster.ids_contained.at(j).id;                                                          

              new_cluster_request.requested_ids.push_back(cluster_element_point);
            }
            new_cluster_request.own_cluster_id = check_cluster_id; 

            requested_cluster_ids.push_back(new_cluster_request);
          }else
          {
            ROS_WARN("No Matching Cluster Detected");
          }
        }   
      }


      if(auction_start == true)
      {
        thr_auction_status = boost::thread(&ExplorationPlanner::respondToAuction, this, requested_cluster_ids, msg.get()->auction_id);
      }

    } else if(msg.get()->auction_status_message == false)
    {
      bool continue_auction = false; 
      if(msg.get()->auction_id == auction_id_number)
      {
        continue_auction = true; 
      }

      if(continue_auction == true)
      {
        //            ROS_INFO("auction_id: %d       local_id: %d", msg.get()->auction_id, 10000*robot_name + auction_id_number);
        bool robot_already_answered = false; 

        for(int i = 0; i < robots_already_responded.size(); i++)
        {
          ROS_INFO("Compare msg name: %d  responded: %d       msg auction: %d   responded: %d", robots_int_name, robots_already_responded.at(i).robot_number, msg.get()->auction_id, robots_already_responded.at(i).auction_number);
          if(robots_int_name == robots_already_responded.at(i).robot_number && msg.get()->auction_id == robots_already_responded.at(i).auction_number)
          {
            ROS_WARN("Same msg already received!!!");
            robot_already_answered = true; 
            break;
          }
        }

        /*
         * Only proceed if the robot has not answered before. 
         */
        if(robot_already_answered == false)
        {
          ROS_INFO("Auction from robot %d received", robot_name);          
          auction_pair_t auction_pair;  
          auction_element_t auction_elements;

          int has_cluster_id = -1;

          /*
           * Visualize the received message
           */
          for(int i = 0; i < msg.get()->available_clusters.size(); i++)
          {
            adhoc_communication::ExpCluster cluster_req; 
            cluster_req = msg.get()->available_clusters.at(i);
            std::string requested_clusters;
            for(int j = 0; j < cluster_req.ids_contained.size(); j++)
            {
              if(j >= 6)
              {
                break;
              }
              requested_clusters.append(NumberToString((int)cluster_req.ids_contained.at(j).id));
              requested_clusters.append(", ");
            }
            ROS_INFO("Received ids: %s", requested_clusters.c_str());
          }



          for(int i = 0; i < msg.get()->available_clusters.size(); i++)
          {
            adhoc_communication::ExpCluster current_cluster;
            current_cluster = msg.get()->available_clusters.at(i);
            int current_cluster_id = checkClustersID(current_cluster);

            ROS_INFO("Received ID converted to cluster ID: %d", current_cluster_id);
            if(current_cluster_id >= 0)
            {
              auction_pair.cluster_id = current_cluster_id;
              auction_pair.bid_value = current_cluster.bid;
              auction_elements.auction_element.push_back(auction_pair);
              ROS_INFO("BID: %f",current_cluster.bid);
            }
          }
          //    ROS_INFO("Robot %d received all bids for all clusters", robot_name);
          auction_elements.robot_id = robots_int_name;                   
          auction_elements.detected_by_robot_str = msg.get()->robot_name;

          auction_mutex.lock();
          auction.push_back(auction_elements);
          auction_mutex.unlock();

          /*
           * A BID is received from one robot. Remember the robot who send the 
           * bid for a special auction id! 
           */      
          //                if(msg.get()->auction_id == 10000*robot_name + auction_id_number)   
          number_of_auction_bids_received++;
          responded_t auction_response; 
          auction_response.auction_number = msg.get()->auction_id;
          auction_response.robot_number = robots_int_name;
          robots_already_responded.push_back(auction_response);
        }else
        {
          ROS_WARN("Robot already answered on this auction");
        }
      }
    }
  }
  auction_running = false;
} 

int ExplorationPlanner::checkClustersID(adhoc_communication::ExpCluster cluster_to_check)
{
  std::vector<compare_pair_t> compare_clusters;  

  std::string requested_clusters;

  for(int j = 0; j < clusters.size(); j++)
  {
    double same_id_found = 0;
    for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
    {
      for(int i = 0; i < cluster_to_check.ids_contained.size(); i++)
      {
        if(clusters.at(j).cluster_element.at(n).id == cluster_to_check.ids_contained.at(i).id)
        {
          same_id_found++;
        }
      }                  
    }
    if(same_id_found > clusters.at(j).cluster_element.size() * 0.4)
    {
      //            ROS_INFO("CLUSTER ID FOUND: %d", clusters.at(j).id);
      return (clusters.at(j).id);
    } 
    return (-1);
  }
}

void ExplorationPlanner::frontierCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{  

  adhoc_communication::ExpFrontierElement frontier_element; 
  for(int i = 0; i < msg.get()->frontier_element.size(); i++)
  {
    frontier_element = msg.get()->frontier_element.at(i);
    bool result = true;
    for (unsigned int j = 0; j < frontiers.size(); j++)
    {
      if(frontier_element.detected_by_robot == robot_name)
      {          
        result = false;
        break;      
      }
      else
      {
        if (frontier_element.id == frontiers.at(j).id)
        {  
          result = false;
          break;
        }
      }
    }
    if (result == true)
    {
      ROS_DEBUG("Received New Frontier of Robot %ld with ID %ld", frontier_element.detected_by_robot, frontier_element.id);
      if(frontier_element.detected_by_robot != robot_name)
      {
        storeFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, "", frontier_element.id); 
      }   
    }
  }
}


void ExplorationPlanner::visited_frontierCallback(const adhoc_communication::ExpFrontier::ConstPtr& msg)
{       
  adhoc_communication::ExpFrontierElement frontier_element; 
  for(int i = 0; i < msg.get()->frontier_element.size(); i++)
  {
    frontier_element = msg.get()->frontier_element.at(i);

    bool result = true;
    for (unsigned int j = 0; j < visited_frontiers.size(); j++)
    {
      if(frontier_element.detected_by_robot == robot_name)
      {    
        result = false;                 
        break;                         
      }
      else
      {
        if (frontier_element.id == visited_frontiers.at(j).id)
        {  
          result = false;
          break;
        }
      }
    }
    if (result == true)
    {
      ROS_DEBUG("Received New Visited Frontier of Robot %ld with ID %ld", frontier_element.detected_by_robot, frontier_element.id);
      if(frontier_element.detected_by_robot != robot_name)
      {    
        storeVisitedFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, "", frontier_element.id); 
      }  
    } 
  }
}

bool ExplorationPlanner::publish_frontier_list()
{
  publish_subscribe_mutex.lock();

  adhoc_communication::ExpFrontier frontier_msg;
  for(int i = 0; i<frontiers.size(); i++)
  {
    adhoc_communication::ExpFrontierElement frontier_element;

    frontier_element.id = frontiers.at(i).id;
    frontier_element.detected_by_robot = frontiers.at(i).detected_by_robot;
    frontier_element.detected_by_robot_str = frontiers.at(i).detected_by_robot_str;
    frontier_element.robot_home_position_x = frontiers.at(i).robot_home_x; 
    frontier_element.robot_home_position_y = frontiers.at(i).robot_home_y;
    frontier_element.x_coordinate = frontiers.at(i).x_coordinate;
    frontier_element.y_coordinate = frontiers.at(i).y_coordinate;

    frontier_msg.frontier_element.push_back(frontier_element);
  }

  sendToMulticast("mc_",frontier_msg, "frontiers");

  publish_subscribe_mutex.unlock();
}

bool ExplorationPlanner::publish_visited_frontier_list()
{
  //    ROS_INFO("PUBLISHING VISITED FRONTIER LIST");

  publish_subscribe_mutex.lock();

  adhoc_communication::ExpFrontier visited_frontier_msg;

  for(int i = 0; i<visited_frontiers.size(); i++)
  {
    adhoc_communication::ExpFrontierElement visited_frontier_element;
    //        visited_frontier_element.id = frontiers.at(i).id;
    visited_frontier_element.detected_by_robot = visited_frontiers.at(i).detected_by_robot;
    visited_frontier_element.detected_by_robot_str = visited_frontiers.at(i).detected_by_robot_str;
    visited_frontier_element.robot_home_position_x = visited_frontiers.at(i).robot_home_x; 
    visited_frontier_element.robot_home_position_y = visited_frontiers.at(i).robot_home_y;
    visited_frontier_element.x_coordinate = visited_frontiers.at(i).x_coordinate;
    visited_frontier_element.y_coordinate = visited_frontiers.at(i).y_coordinate;

    visited_frontier_msg.frontier_element.push_back(visited_frontier_element);
  }

  //    pub_visited_frontiers.publish(visited_frontier_msg);
  sendToMulticast("mc_",visited_frontier_msg, "visited_frontiers");

  publish_subscribe_mutex.unlock();
}

/*
 * Check if the next goal is efficient enough to steer the robot to it.
 * If it is a goal, which has previously be seen, it is not required
 * to visit this goal again.
 * make sure that there are no failures in calculation! Therefore
 * this plausibility check is done. Only values of less then 50m
 * are valid. (Calculation of coordinates in the costmap often
 * produce very big values which are miss-interpretations)
 */
bool ExplorationPlanner::check_efficiency_of_goal(double x, double y) {

  //    ROS_INFO("Check efficiency");
  double diff_home_x = visited_frontiers.at(0).x_coordinate - x;
  double diff_home_y = visited_frontiers.at(0).y_coordinate - y;

  if (fabs(diff_home_x) <= MAX_DISTANCE && fabs(diff_home_y) <= MAX_DISTANCE) 
  {
    for (int i = 0; i < visited_frontiers.size(); i++)
    {
      /*
       * Calculate the distance between all previously seen goals and the new
       * found frontier!!
       */
      double diff_x = visited_frontiers.at(i).x_coordinate - x;
      double diff_y = visited_frontiers.at(i).y_coordinate - y;

      if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE) {
        ROS_DEBUG("x: %f  y: %f too close to visited at x: %f   y: %f   dif_x: %f   dif_y: %f", x, y, visited_frontiers.at(i).x_coordinate, visited_frontiers.at(i).y_coordinate, diff_x, diff_y);
        return false;
      }
    }
    for (int i = 0; i < unreachable_frontiers.size(); i++)
    {
      /*
       * Calculate the distance between all previously seen goals and the new
       * found frontier!!
       */
      double diff_x = unreachable_frontiers.at(i).x_coordinate - x;
      double diff_y = unreachable_frontiers.at(i).y_coordinate - y;

      if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE) {
        ROS_DEBUG("x: %f  y: %f too close to unreachable at x: %f   y: %f   dif_x: %f   dif_y: %f", x, y, unreachable_frontiers.at(i).x_coordinate, unreachable_frontiers.at(i).y_coordinate, diff_x, diff_y);
        return false;
      }
    }
    return true;
  }
  else
  {
    ROS_WARN("OUT OF HOME RANGE");
    return false;
  }
}

void ExplorationPlanner::clearVisitedAndSeenFrontiersFromClusters()
{
  ROS_INFO("Clear VisitedAndSeenFrontiers from Cluster");
  std::vector<int> goals_to_clear;

  for(int i = 0; i < clusters.size(); i++)
  {
    for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
    {
      /* Now iterate over all frontiers and check if cluster elements are 
       * still available in the frontier vector
       */
      bool cluster_still_valid = false; 
      for(int m = 0; m < frontiers.size(); m++)
      {
        if(clusters.at(i).cluster_element.at(j).id == frontiers.at(m).id)
        {
          /*Cluster is still valid, do not clear it*/
          cluster_still_valid = true; 
          break;
        }
      }
      if(cluster_still_valid == false)
      {
        clusters.at(i).cluster_element.erase(clusters.at(i).cluster_element.begin() +j);
        if(j > 0)
          j--;                  
      }
    }
  }

  for(int i = 0; i < clusters.size(); i++)
  {
    if(clusters.at(i).cluster_element.size() <= 0)
    {
      clusters.erase(clusters.begin() + i);
    }
  }

  ROS_INFO("Done");
}

void ExplorationPlanner::clearVisitedFrontiers()
{
  /* visited_frontier.at(0) is the Home point. Do not compare
   * with this point. Otherwise this algorithm deletes it, a new
   * frontier close to the home point is found which is then again
   * deleted since it is to close to the home point. This would not
   * have any impact on the exploration, but in simulation mode 
   * (simulation = true) the frontier_ID is steadily up counted. 
   * This is not necessary! 
   */
  std::vector<int> goals_to_clear;

  for (int i = 1; i < visited_frontiers.size(); i++)
  {
    for (int j = 0; j < frontiers.size(); j++)
    {
      double diff_x = visited_frontiers.at(i).x_coordinate - frontiers.at(j).x_coordinate;
      double diff_y = visited_frontiers.at(i).y_coordinate - frontiers.at(j).y_coordinate;

      if (fabs(diff_x) <= MAX_GOAL_RANGE && fabs(diff_y) <= MAX_GOAL_RANGE) {
        removeStoredFrontier(frontiers.at(j).id, "");
        if(j > 0)
        {
          j--;
        }
        break;
      }
    }
  }
}

void ExplorationPlanner::clearUnreachableFrontiers()
{
  /* visited_frontier.at(0) is the Home point. Do not compare
   * with this point. Otherwise this algorithm deletes it, a new
   * frontier close to the home point is found which is then again
   * deleted since it is to close to the home point. This would not
   * have any impact on the exploration, but in simulation mode 
   * (simulation = true) the frontier_ID is steadily up counted. 
   * This is not necessary! 
   */
  std::vector<int> goals_to_clear;

  for (int i = 1; i < unreachable_frontiers.size(); i++)
  {
    for (int j = 0; j < frontiers.size(); j++)
    {
      double diff_x = unreachable_frontiers.at(i).x_coordinate - frontiers.at(j).x_coordinate;
      double diff_y = unreachable_frontiers.at(i).y_coordinate - frontiers.at(j).y_coordinate;

      if (fabs(diff_x) <= 0.2 && fabs(diff_y) <= 0.2) {
        //                goals_to_clear.push_back(frontiers.at(j).id);
        removeStoredFrontier(frontiers.at(j).id, "");
        if(j > 0)
        {
          j--;
        }
        break;
      }
    }
  }
}

void ExplorationPlanner::clearSeenFrontiers(costmap_2d::Costmap2DROS *global_costmap)
{
  unsigned int mx, my, point;
  std::vector<int> neighbours, goals_to_clear;

  for(int i = 0; i < frontiers.size(); i++)
  {
    bool unknown_found = false; 
    bool obstacle_found = false;
    bool freespace_found = false;

    mx = 0; 
    my = 0;

    if(!global_costmap->getCostmap()->worldToMap(frontiers.at(i).x_coordinate,frontiers.at(i).y_coordinate,mx,my))
    {
      ROS_ERROR("Cannot convert coordinates successfully.");
      continue;
    }

    int neighbours[8];
    getAdjacentPoints(global_costmap->getCostmap()->getIndex(mx,my), neighbours);

    for (int j = 0; j < 8; j++)
    {
      if (occupancy_grid_array_[neighbours[i]] == costmap_2d::NO_INFORMATION) 
      {
        unknown_found = true;
      }
      else if(occupancy_grid_array_[neighbours[i]] == costmap_2d::FREE_SPACE)
      {
        freespace_found = true;
      }
      else if(occupancy_grid_array_[neighbours[i]] == costmap_2d::LETHAL_OBSTACLE)
      {
        obstacle_found = true;
      }
    } 

    if(unknown_found == false || obstacle_found == true || freespace_found == false)
    {
      removeStoredFrontier(frontiers.at(i).id, "");
      if(i > 0)
        i--;
      seen_frontier_list.push_back(frontiers.at(i));
    }
  }
}

/*
 * searches the occupancy grid for frontier cells and merges them into one target point per frontier.
 * The returned frontiers are in world coordinates.
 */
void ExplorationPlanner::findFrontiers() {
  // TODO:  fix this, it's a little inefficient, 2 hrs
  ROS_INFO("called find frontiers");

  /** beginnings of BFS search for frontiers
  if (!costmap_ros_->getRobotPose(robotPose)) {
    ROS_WARN("could not get robot pose");
  }

  int robx = robotPose.getOrigin().getX();
  int roby = robotPose.getOrigin().getY();

  boost::array<int, 2> lengths = { { map_width_, map_height_ } };
  boost::grid_graph<2> graph(lengths);
  */

  allFrontiers.clear();
  int select_frontier = 1;
  std::vector<double> final_goal,start_points;

  int new_frontier_point = 0;
  for (unsigned int i = 0; i < num_map_cells_; i++) {

    int new_frontier_point = isFrontier(i);
    if (new_frontier_point != 0) {
      allFrontiers.push_back(new_frontier_point);
    }
  }
  ROS_INFO("Found %lu frontier cells which are transformed into frontiers points. Starting transformation...", allFrontiers.size());

  for (unsigned int i = 0; i < allFrontiers.size(); ++i) {

    geometry_msgs::PoseStamped finalFrontier;
    double wx, wy;
    unsigned int mx, my;
    bool result;

    costmap_ros_->getCostmap()->indexToCells(allFrontiers.at(i), mx, my);
    costmap_ros_->getCostmap()->mapToWorld(mx, my, wx, wy);

    result = true;
    // checks to make sure frontiers are far enough apart to store
    for (unsigned int j = 0; j < frontiers.size(); j++)
    {
      if (fabs(wx - frontiers.at(j).x_coordinate) <= MAX_GOAL_RANGE && fabs(wy - frontiers.at(j).y_coordinate) <= MAX_GOAL_RANGE) 
      {  
        result = false;
        break;
      }
    }
    if (result == true)
    {
      storeFrontier(wx,wy,robot_name,robot_str,-1);
    }
  }

  ROS_INFO("Size of all frontiers in the list: %lu", frontiers.size());
}

bool ExplorationPlanner::auctioning(
    std::vector<double> *final_goal, 
    std::vector<int> *clusters_available_in_pool, 
    std::vector<std::string> *robot_str_name)
{

  ROS_INFO("Start Auctioning");

  /*
   * Check if Auction is running and wait until it is finished
   */
  int timer_count = 0;
  number_of_auction_bids_received = 0;
  std::string robo_name;

  float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  float wait_if_auction_runs = r * 2;

  if(wait_if_auction_runs < 0)
    wait_if_auction_runs = wait_if_auction_runs * (-1);

  ROS_INFO("Waiting %f second if a auction is running",wait_if_auction_runs);
  ros::Duration(wait_if_auction_runs).sleep();

  while(auction_running)
  {
    ROS_INFO("Waiting %f second because an auction is currently running",wait_if_auction_runs);
    ros::Duration(wait_if_auction_runs).sleep();
    float wait_if_auction_runs = r * 2;
    wait_if_auction_runs = wait_if_auction_runs - robot_name;
    if(wait_if_auction_runs < 0)
      wait_if_auction_runs = wait_if_auction_runs * (-1);
  }

  if(auction_running)
  {
    ROS_INFO("Auction is running, leaving method");
    return false;
  }
  else
  {
    ROS_INFO("No auction is running, clearing");
    auction.clear();
  }

  adhoc_communication::ExpAuction reqest_clusters, auction_msg;

  auction_msg.start_auction = true; 
  auction_msg.auction_finished = false;
  auction_msg.auction_status_message = true;

  std::stringstream ss;
  ss << robot_name; 
  std::string prefix = "";
  robo_name = prefix.append(ss.str());    

  auction_msg.robot_name = robo_name;

  for(int i = 0; i < clusters.size(); i++)
  {
    adhoc_communication::ExpCluster cluster_request;


    for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
    { 
      adhoc_communication::ExpClusterElement cluster_request_element;
      cluster_request_element.id = clusters.at(i).cluster_element.at(j).id;
      cluster_request.ids_contained.push_back(cluster_request_element);
    }
    //        auction_status.requested_clusters.push_back(cluster_request);
    auction_msg.requested_clusters.push_back(cluster_request);
  }

  auction_msg.auction_id = 10000*robot_name + auction_id_number;

  std::string numbers_of_operating_robots; 

  ROS_INFO("Robot %d starting an auction", robot_name);

  sendToMulticastAuction("mc_", auction_msg, "auction");
  ros::Duration(3).sleep();

  ROS_INFO("number of auction bids received: %d", number_of_auction_bids_received);

  if(number_of_auction_bids_received > 0)
    number_of_completed_auctions++;
  else
    number_of_uncompleted_auctions++;

  /*
   * Select the best cluster for yourself
   */
  ROS_INFO("Selecting the most attractive cluster");
  bool cluster_selected_flag = selectClusterBasedOnAuction(final_goal, clusters_available_in_pool, robot_str_name);

  /*
   * Stop the auction
   */
  ROS_INFO("Stop the auction");

  auction_msg.start_auction = false; 
  auction_msg.auction_finished = true;
  auction_msg.auction_status_message = true;   
  auction_msg.robot_name = robo_name;
  auction_msg.requested_clusters.clear();

  if(cluster_selected_flag == true)
  {
    /*
     * Tell the others which cluster was selected
     */
    std::vector<transform_point_t> occupied_ids;
    clusterIdToElementIds(final_goal->at(4), &occupied_ids);
    for(int i = 0; i < occupied_ids.size(); i++)
    {
      //            auction_status.occupied_ids.push_back(occupied_ids.at(i));
      adhoc_communication::ExpAuctionElement auction_element;
      auction_element.id = occupied_ids.at(i).id;
      auction_element.detected_by_robot_str = occupied_ids.at(i).robot_str;
      auction_msg.occupied_ids.push_back(auction_element);
    }        
  }
  sendToMulticastAuction("mc_", auction_msg, "auction");

  first_run = false; 
  auction_id_number++;

  ROS_INFO("return %d", cluster_selected_flag);
  return (cluster_selected_flag);
}



bool ExplorationPlanner::clusterIdToElementIds(int cluster_id, std::vector<transform_point_t>* occupied_ids)
{
  for(int i = 0; i < clusters.size(); i++)
  {
    if(clusters.at(i).id == cluster_id)
    {
      for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
      {
        transform_point_t point;
        point.id = clusters.at(i).cluster_element.at(j).id;
        point.robot_str = clusters.at(i).cluster_element.at(j).detected_by_robot_str;

        occupied_ids->push_back(point);                
      }
    }
  }
}

std::vector< std::vector<int> > array_to_matrix(int* m, int rows, int cols) {
  int i,j;
  std::vector< std::vector<int> > r;
  r.resize(rows, std::vector<int>(cols, 0));

  for(i=0;i<rows;i++)
  {
    for(j=0;j<cols;j++)
      r[i][j] = m[i*cols+j];
  }
  return r;
}


bool ExplorationPlanner::selectClusterBasedOnAuction(
    std::vector<double> *goal, 
    std::vector<int> *cluster_in_use_already_count, 
    std::vector<std::string> *robot_str_name_to_return)
{
  ROS_INFO("Select the best cluster based on auction bids");

  int own_row_to_select_cluster = 0;
  int own_column_to_select = 0; 

  /*
   * Select the method to select clusters from the matrix.
   * 0 ... select nearest cluster from OWN calculations
   * 1 ... gather information of others, estimate trajectory length based on
   *       distance information and select the optimal cluster using the 
   *       KuhnMunkres algorithm.
   * 2 ... select
   */
  int method_used = 2;

  int count = 0;
  int auction_cluster_element_id = -1;

  auction_element_t auction_elements; 
  auction_pair_t auction_pair;

  ROS_INFO("Cluster Size: %lu", clusters.size());
  for(int i = 0; i < clusters.size(); i++)
  {        
    int my_auction_bid = calculateAuctionBID(
        clusters.at(i).id, 
        trajectory_strategy,
        robotPose.getOrigin().getX(),
        robotPose.getOrigin().getY());
    if(my_auction_bid == -1)
    {
      ROS_WARN("Own BID calculation failed");
    }
    auction_pair.cluster_id = clusters.at(i).id;
    auction_pair.bid_value = my_auction_bid;
    auction_elements.auction_element.push_back(auction_pair);        
  }
  auction_elements.robot_id = robot_name;
  auction.push_back(auction_elements);

  /*
   * Remove all clusters which have previously been selected by other robots
   */
  std::vector<int> clusters_to_erase;
  for(int j = 0; j < clusters.size(); j++)
  {           
    for(int i = 0; i < already_used_ids.size(); i++)
    {
      if(already_used_ids.at(i) == clusters.at(j).id)
      {
        /*Already used cluster found, therefore erase it*/
        for(int m = 0; m < auction.size(); m++)
        {
          for(int n = 0; n < auction.at(m).auction_element.size(); n++)
          {
            if(auction.at(m).auction_element.at(n).cluster_id == already_used_ids.at(i))
            {
              ROS_INFO("Already used Cluster %d found .... unconsider it",auction.at(m).auction_element.at(n).cluster_id);
              auction.at(m).auction_element.at(n).bid_value = 10000;
            }
          }
        }
      }
    }
  }

  ROS_INFO("Matrix");
  /*
   * Calculate the matrix size
   */
  int col = 0, row = 0;

  int robots_operating; 
  nh.param<int>("robots_in_simulation",robots_operating, 1);
  ROS_INFO("robots operating: %d", robots_operating);

  row = auction.size();
  col = clusters.size();
  ROS_INFO("robots: %d     clusters: %d", row, col);

  /*
   * create a matrix with boost library
   */
  int max_size = std::max(col,row);
  boost::numeric::ublas::matrix<double> m(max_size,max_size);

  ROS_INFO("Empty the matrix");
  /*
   * initialize the matrix with all -1
   */
  for(int i = 0; i < m.size1(); i++)
  {
    for(int j = 0; j < m.size2(); j++)
    {
      m(i,j) = 0;
    }
  }

  ROS_INFO("Fill matrix");
  /*
   * Now fill the matrix with real auction values and use 
   * a method for the traveling salesman problem
   * 
   * i ... number of robots
   * j ... number of clusters
   */   
  //    ROS_INFO("robots: %d   clusters: %d", row, col);
  for(int i = 0; i < row; i++)
  {
    ROS_INFO("                 ");
    ROS_INFO("****** i: %d   auction size: %lu ******", i, auction.size());
    for(int j = 0; j < col; j++)
    {     
      ROS_INFO("j: %d   cluster size: %lu", j, clusters.size());
      bool found_a_bid = false;
      bool cluster_valid_flag = false; 

      for(int n = 0; n < auction.at(i).auction_element.size(); n++)
      {            
        cluster_valid_flag = true; 
        if(clusters.at(j).id == auction.at(i).auction_element.at(n).cluster_id && auction.at(i).auction_element.at(n).bid_value >= 0)
        {
          m(j,i) = auction.at(i).auction_element.at(n).bid_value;
          found_a_bid = true; 
          break;
        }
      }

      ROS_INFO("Cluster elements checked, found BID: %d", found_a_bid);

      /*
       * The auction does not contain a BID value for this cluster,
       * therefore try to estimate it, using the other robots position. 
       */
      if(found_a_bid == false) // && cluster_valid_flag == true)
      {
        ROS_INFO("No BID received ...");
        int distance = -1;
        other_robots_position_x = -1;
        other_robots_position_y = -1;

        ROS_INFO("---- clusters: %lu element size: %lu  robots positions: %lu ----", clusters.size(), clusters.at(j).cluster_element.size(), other_robots_positions.positions.size());
        for(int d = 0; d < clusters.at(j).cluster_element.size(); d++)
        {
          ROS_INFO("Access clusters.at(%d).cluster_element.at(%d)", j, d);
          position_mutex.lock();
          /*Check position of the current robot (i)*/
          for(int u = 0; u < other_robots_positions.positions.size(); u++)
          {
            adhoc_communication::MmPoint position_of_robot; 
            position_of_robot = other_robots_positions.positions.at(u); 

            int robots_int_id = atoi(position_of_robot.src_robot.substr(6,1).c_str());
            ROS_INFO("Robots int id: %d", robots_int_id);
            if(auction.at(i).robot_id == robots_int_id)
            {
              other_robots_position_x = position_of_robot.x;
              other_robots_position_y = position_of_robot.y;

              ROS_INFO("Robots %d     position_x: %f     position_y: %f", robots_int_id, other_robots_position_x, other_robots_position_y);
              break;
            }else
            {
              ROS_ERROR("Unable to look up robots position!");
            }                        
          }
          position_mutex.unlock();

          if(other_robots_position_x != -1 && other_robots_position_y != -1)
          {
            m(j,i) =  calculateAuctionBID(
                clusters.at(i).id, 
                trajectory_strategy,
                other_robots_position_x,
                other_robots_position_y);
          } else {
            m(j,i) = 10000;
          }
        }
        ROS_INFO("Column filled");
      }
      ROS_INFO("No columns left. Check next robot");
    }  
  }

  ROS_INFO("Completed");

  /*
   * If the number of clusters is less the number of robots, 
   * the assigning algorithmn would not come to a solution. Therefore
   * select the nearest cluster 
   */
  if(col < row)
  {
    ROS_ERROR("Number of clusters is less the number of robots. Select the closest");
    if(col == 0)
    {
      ROS_ERROR("No cluster anymore available");
      /*No clusters are available at all*/
      return false;
    }
    method_used = 0; 
  }

  /*
   * Select the strategy how to select clusters from the matrix, put it in auction_cluster_element_id
   * 
   * 0 ... select the last one on the matrix
   * 1 ... 
   */
  if(method_used == 0)
  {
    ROS_INFO("Columns left: %d", col);
    if(auction.size() > 0)
      if(auction.back().auction_element.size() > 0)
        auction_cluster_element_id = auction.back().auction_element.front().cluster_id; 
  }
  if(method_used == 1)
  {
    /*
     * Use the Ungarische method/ KuhnMunkresAlgorithm in order to figure out which 
     * cluster the most promising one is for myself
     */

    /* an example cost matrix */
    int mat[col*row];
    int counter = 0; 
    for(int j = 0; j < col; j++)
    {
      for(int i = 0; i < row; i++)
      { 
        mat[counter] = m(j,i);
        counter++;
      }
    }

    std::vector< std::vector<int> > m2 = array_to_matrix(mat,col,row);

    /*
     * Last row in the matrix contains BIDs of the robot itself
     * Remember the max row count before filling up with zeros. 
     */
    own_row_to_select_cluster = row-1;
    ROS_INFO("Own row: %d", own_row_to_select_cluster);

    /* an example cost matrix */
    int r[3*3] =  {14,15,15,30,1,95,22,14,12};
    std::vector< std::vector<int> > m3 = array_to_matrix(r,3,3);

    /* initialize the gungarian_problem using the cost matrix*/
    Hungarian hungarian(m2, col, row, HUNGARIAN_MODE_MINIMIZE_COST);

    //        Hungarian hungarian(m3, 3, 3, HUNGARIAN_MODE_MINIMIZE_COST);

    fprintf(stderr, "cost-matrix:");
    hungarian.print_cost();

    /* solve the assignment problem */
    for(int i = 0; i < 5; i++)
    {
      if(hungarian.solve() == true)
        break;
    }

    const std::vector< std::vector<int> > assignment = hungarian.assignment();       

    /* some output */
    fprintf(stderr, "assignment:");
    hungarian.print_assignment();

    for(int i = 0; i < assignment.size(); i++)
    {
      if(assignment.at(i).at(own_row_to_select_cluster) == 1)
      {
        auction_cluster_element_id = clusters.at(i).id;
        ROS_INFO("Selected Cluster at position : %d   %d",i ,own_row_to_select_cluster);
        break;
      }
    }

  }
  if(method_used == 2)
  {
    Matrix<double> mat = convert_boost_matrix_to_munkres_matrix<double>(m);
    ROS_INFO("Matrix (%ux%u):",mat.rows(),mat.columns());

    // Display begin matrix state.
    for ( int new_row = 0 ; new_row < mat.rows(); new_row++ ) {
      if(new_row > 9)
      {
        int rows_left = mat.rows() - new_row + 1;
        std::cout << "... (" << rows_left << " more)";
        break;
      }
      for ( int new_col = 0 ; new_col < mat.columns(); new_col++ ) {
        if(new_col > 9)
        {
          int columns_left = mat.columns() - new_col + 1;
          std::cout << "... (" << columns_left << " more)";
          break;
        }
        std::cout.width(2);
        std::cout << mat(new_row,new_col) << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;

    Munkres munk;
    munk.solve(mat);

    ROS_INFO("Solved :");
    // Display solved matrix.
    for ( int new_row = 0 ; new_row < mat.rows(); new_row++ ) {
      if(new_row > 9)
      {
        int rows_left = mat.rows() - new_row + 1;
        std::cout << "... (" << rows_left << " more)";
        break;
      }
      for ( int new_col = 0 ; new_col < mat.columns(); new_col++ ) {
        if(new_col > 9)
        {
          int columns_left = mat.columns() - new_col + 1;
          std::cout << "... (" << columns_left << " more)";
          break;
        }
        std::cout.width(2);
        std::cout << mat(new_row,new_col) << " ";
      }
      std::cout << std::endl;
    }

    std::cout << std::endl;

    own_row_to_select_cluster = row-1;
    for(int i = 0; i < mat.columns(); i++)
    {
      if(mat(i,own_row_to_select_cluster) == 1)
      {
        auction_cluster_element_id = clusters.at(i).id;
        own_column_to_select = i; 
        ROS_INFO("Selected Cluster at position : %d   %d  with BID: %f",i ,own_row_to_select_cluster, mat(i, own_row_to_select_cluster));
        break;
      }
    }
  }

  /*
   * Try to navigate to the selected cluster. If it failes, take the next efficient
   * cluster and try again
   */
  if(auction_cluster_element_id != -1)
  {  

    /*
     * Select the above calculated goal. If this cluster is still in use,
     * set this cluster in the matrix to zero and restart the auction
     *
     *
     * Following should just be executed if the selection using auctioning 
     * does fail
     */
    while(ros::ok())
    {
      ROS_INFO("Try to determining goal");
      std::vector<double> new_goal;
      std::vector<std::string> robot_str_name;
      bool goal_determined = determine_goal(6, &new_goal, count, auction_cluster_element_id, &robot_str_name);

      if(goal_determined == true)
      {
        double determined_goal_id = new_goal.at(4);
        bool used_cluster = false;
        for(int m = 0; m < already_used_ids.size(); m++)
        {            
          if(determined_goal_id == already_used_ids.at(m))
          {
            ROS_INFO("Cluster in use already");
            cluster_in_use_already_count->push_back(1);
            used_cluster = true; 
            count++;
            break;
          }     
        }  
        if(used_cluster == false)
        {
          ROS_INFO("Cluster was not used by any robot before");
          goal->push_back(new_goal.at(0));
          goal->push_back(new_goal.at(1));
          goal->push_back(new_goal.at(2));
          goal->push_back(new_goal.at(3));
          goal->push_back(new_goal.at(4));

          robot_str_name_to_return->push_back(robot_str_name.at(0));
          return true; 
        }
      }else
      {      
        if(clusters.size() <= count)
        {
          ROS_INFO("Not possible to select any goal from the available clusters");
          return false;
        }else
        {
          ROS_INFO("Current cluster empty ... select next one");
        }
        count++;
      }
    }   
    return false;
  }else
  {
    /*
     * No auction elements are available. Auctioning has failed
     */

    ROS_INFO("Auction has failed, no auction elements are existent. Choosing nearest cluster");
    std::vector<double> new_goal;
    std::vector<std::string> robot_str_name;
    bool goal_determined = determine_goal(4, &new_goal, count, -1, &robot_str_name);
    if(goal_determined == true)
    {
      goal->push_back(new_goal.at(0));
      goal->push_back(new_goal.at(1));
      goal->push_back(new_goal.at(2));
      goal->push_back(new_goal.at(3));
      goal->push_back(new_goal.at(4));
      return true; 
    }else
    {
      return false;
    }
  }
  return false; 
}



bool ExplorationPlanner::negotiate_Frontier(double x, double y, int detected_by, int id, int cluster_id_number)
{

  ROS_INFO("Negotiating Frontier with id: %d  at Cluster: %d", id, cluster_id_number);

  // get index of cluster id number
  int cluster_vector_position = 0;
  for (int i = 0; i < clusters.size(); i++)
  {
    if(clusters.at(i).id == cluster_id_number)
    {
      cluster_vector_position = i;
      break;
    }
  }          

  ROS_DEBUG("cluster vector position: %d", cluster_vector_position);

  bool entry_found = false;
  bool id_in_actual_cluster = false;

  for(int i = 0; i < negotiation_list.size(); i++)
  {
    for(int k = 0; k < clusters.at(cluster_vector_position).cluster_element.size(); k++)
    {
      id_in_actual_cluster = false;
      if(negotiation_list.at(i).id == clusters.at(cluster_vector_position).cluster_element.at(k).id)
      {     
        ROS_INFO("         Same ID detected");   
        for(int j = 0; j < clusters.at(cluster_vector_position).cluster_element.size(); j++)
        {
          for(int m = 0; m < my_negotiation_list.size(); m++)
          {
            if(clusters.at(cluster_vector_position).cluster_element.at(j).id == my_negotiation_list.at(m).id)
            {
              ROS_INFO("         But is in the current working cluster! everything alright");
              id_in_actual_cluster = true;
              return true;
            }
          }
        }

        double used_cluster_ids = 0;
        if(id_in_actual_cluster == false)
        {
          ROS_INFO("Checking how many goals in the cluster are already occupied");
          for(int n = 0; n < negotiation_list.size(); n++)
          {
            for(int m = 0; m < clusters.at(cluster_vector_position).cluster_element.size(); m++)
            {
              if(negotiation_list.at(n).id == clusters.at(cluster_vector_position).cluster_element.at(m).id)
              {
                used_cluster_ids++;
              }
            }
          } 
          //                    ROS_INFO("%f goals of %f in the cluster  -> %f",used_cluster_ids, (double)clusters.at(cluster_vector_position).cluster_element.size(), double(used_cluster_ids / (double)clusters.at(cluster_vector_position).cluster_element.size()));
          if(double(used_cluster_ids / (double)clusters.at(cluster_vector_position).cluster_element.size()) >= 0.1)
          {
            ROS_INFO("Negotiation failed the other Robot got cluster %d already", cluster_id_number);
            entry_found = true;
            return false;
          }
        }
      }
    }
  }
  if(entry_found == false)
  {
    frontier_t new_frontier;
    new_frontier.x_coordinate = x;
    new_frontier.y_coordinate = y;
    new_frontier.detected_by_robot = detected_by;
    new_frontier.id = id;

    publish_negotiation_list(new_frontier, cluster_id_number);

    return true;
  }     
  return false;
}


bool ExplorationPlanner::determine_goal(
    int strategy, 
    std::vector<double> *final_goal, 
    int count, 
    int actual_cluster_id, 
    std::vector<std::string> *robot_str_name)
{
  if (!costmap_ros_->getRobotPose(robotPose))
  {
    ROS_ERROR("Failed to get RobotPose");
  }
  if(strategy == 1)
  {
    for (int i = frontiers.size() - 1 - count; i >= 0; i--)
    {
      if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
      {
        ROS_INFO("Determined frontier with ID: %d   at x: %f     y: %f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

        /*
         * Solve the Problem of planning a path to a frontier which is located
         * directly "in" the obstacle. Therefore back-off 5% of the targets
         * coordinate. Since the direction of x and y can be positive and
         * negative either, multiply the coordinate with 0.95 to get 95% of its
         * original value.
         */
        final_goal->push_back(frontiers.at(i).x_coordinate); 
        final_goal->push_back(frontiers.at(i).y_coordinate);
        final_goal->push_back(frontiers.at(i).detected_by_robot);
        final_goal->push_back(frontiers.at(i).id);

        robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
        return true;
      }
    }
    return false;
  }
  else if(strategy == 2)
  {
    for (int i = 0 + count; i < frontiers.size(); i++)
    {                    
      if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
      {
        ROS_INFO("Determined frontier with ID: %d   at x: %f     y: %f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

        final_goal->push_back(frontiers.at(i).x_coordinate);
        final_goal->push_back(frontiers.at(i).y_coordinate);
        final_goal->push_back(frontiers.at(i).detected_by_robot);
        final_goal->push_back(frontiers.at(i).id);

        robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
        return true;
      }
    }
  }    
  else if(strategy == 3)
  {
    while(true)
    {
      if(frontiers.size() > 0)
      {

        int i = int(frontiers.size()*rand()/(RAND_MAX));

        ROS_INFO("Random frontier ID: %d", frontiers.at(i).id);

        if (check_efficiency_of_goal(frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate) == true)
        {
          ROS_INFO("Determined frontier with ID: %d   at x: %f     y: %f   detected by Robot %d", frontiers.at(i).id, frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate, frontiers.at(i).detected_by_robot);

          final_goal->push_back(frontiers.at(i).x_coordinate);
          final_goal->push_back(frontiers.at(i).y_coordinate);
          final_goal->push_back(frontiers.at(i).detected_by_robot);
          final_goal->push_back(frontiers.at(i).id);

          robot_str_name->push_back(frontiers.at(i).detected_by_robot_str);
          return true;
        }
      }
      break;
    }  
  }
  else if(strategy == 4)
  {
    int cluster_vector_position = 0;

    if(actual_cluster_id != -1)
    {
      if(clusters.size() > 0)
      {                   
        for (int i = 0; i < clusters.size(); i++)
        {
          if(clusters.at(i).id == actual_cluster_id)
          {
            if(clusters.at(i).cluster_element.size() > 0)
            {
              cluster_vector_position = i; 
            }
            break;
          }
        }
      }
    }

    ROS_INFO("Calculated vector position of cluster %d", actual_cluster_id);   
    /*
     * Iterate over all clusters .... if cluster_vector_position is set
     * also the clusters with a lower have to be checked if the frontier
     * determination fails at clusters.at(cluster_vector_position). therefore
     * a ring-buffer is operated to iterate also over lower clusters, since
     * they might have changed.
     */
    int nothing_found_in_actual_cluster = 0;
    int visited_clusters = 0;
    for (int i = 0 + count; i < clusters.size(); i++)
    {
      //                    ROS_INFO("Cluster vector: %d  i: %d ", cluster_vector_position, i);
      i = i+ cluster_vector_position;
      i = i % (clusters.size());
      for (int j = 0; j < clusters.at(i).cluster_element.size(); j++)
      {                    
        if (check_efficiency_of_goal(clusters.at(i).cluster_element.at(j).x_coordinate, clusters.at(i).cluster_element.at(j).y_coordinate) == true)
        {
          ROS_INFO("------------------------------------------------------------------");
          ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(i).cluster_element.at(j).id, (int)clusters.at(i).id);

          final_goal->push_back(clusters.at(i).cluster_element.at(j).x_coordinate);
          final_goal->push_back(clusters.at(i).cluster_element.at(j).y_coordinate);
          final_goal->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot);
          final_goal->push_back(clusters.at(i).cluster_element.at(j).id);

          // number of the cluster we operate in
          final_goal->push_back(clusters.at(i).id);
          robot_str_name->push_back(clusters.at(i).cluster_element.at(j).detected_by_robot_str);
          return true;
        }

      }

      nothing_found_in_actual_cluster ++;
      visited_clusters ++;

      if(nothing_found_in_actual_cluster == 1)
      {
        //start again at the beginning(closest cluster))
        i=0;
        cluster_vector_position = 0;
      }

      if(visited_clusters == clusters.size())
      {
        break;
      }
    }               
  }
  else if(strategy == 5)
  {
    int cluster_vector_position = 0;
    bool cluster_could_be_found = false; 

    if(actual_cluster_id != -1)
    {
      if(clusters.size() > 0)
      {                   
        for (int i = 0; i < clusters.size(); i++)
        {
          if(clusters.at(i).id == actual_cluster_id)
          {
            if(clusters.at(i).cluster_element.size() > 0)
            {
              cluster_vector_position = i; 
              cluster_could_be_found = true;
              break;
            }
          }
        }                      
      }
      if(cluster_could_be_found == false)
      {
        /*
         * The cluster we operated in is now empty
         */
        return false; 
      }
    }
    else if(actual_cluster_id == -1)
    {
      // No cluster was previously selected
      final_goal->push_back(-1.0);
      return false;
    }
    ROS_INFO("Calculated vector position: %d of cluster %d", cluster_vector_position, actual_cluster_id);   
    /*
     * Iterate over all clusters .... if cluster_vector_position is set
     * also the clusters with a lower have to be checked if the frontier
     * determination fails at clusters.at(cluster_vector_position). therefore
     * a ring-buffer is operated to iterate also over lower clusters, since
     * they might have changed.
     */
    int nothing_found_in_actual_cluster = 0;
    int visited_clusters = 0;

    //                if(clusters.size() > 0)
    //                {

    int position = (cluster_vector_position +count) % (clusters.size());
    for (int j = 0; j < clusters.at(position).cluster_element.size(); j++)
    {     
      if(count >= clusters.size())
      {
        break;
      }

      if(clusters.at(position).cluster_element.size() > 0)
      {
        if (check_efficiency_of_goal(clusters.at(position).cluster_element.at(j).x_coordinate, clusters.at(position).cluster_element.at(j).y_coordinate) == true)
        {
          ROS_INFO("------------------------------------------------------------------");
          ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(position).cluster_element.at(j).id, (int)clusters.at(position).id);

          final_goal->push_back(clusters.at(position).cluster_element.at(j).x_coordinate);
          final_goal->push_back(clusters.at(position).cluster_element.at(j).y_coordinate);
          final_goal->push_back(clusters.at(position).cluster_element.at(j).detected_by_robot);
          final_goal->push_back(clusters.at(position).cluster_element.at(j).id);

          // number of the cluster we operate in
          final_goal->push_back(clusters.at(position).id);
          robot_str_name->push_back(clusters.at(position).cluster_element.at(j).detected_by_robot_str);
          return true;
        }
      }

    }

    //                }
    //                ROS_INFO("No clusters are available anymore");

    return false;                                
  }
  else if(strategy == 6)
  {
    int cluster_vector_position = 0;
    bool cluster_could_be_found = false; 

    if(actual_cluster_id != -1)
    {
      if(clusters.size() > 0)
      {                   
        for (int i = 0; i < clusters.size(); i++)
        {
          if(clusters.at(i).id == actual_cluster_id)
          {
            if(clusters.at(i).cluster_element.size() > 0)
            {
              cluster_vector_position = i; 
              cluster_could_be_found = true;
              break;
            }
          }
        }                      
      }
      if(cluster_could_be_found == false)
      {
        /*
         * The cluster we operated in is now empty
         */
        return false; 
      }
    }
    else if(actual_cluster_id == -1)
    {
      // No cluster was previously selected
      final_goal->push_back(-1.0);
      return false;
    }
    ROS_INFO("Calculated vector position: %d of cluster %d", cluster_vector_position, actual_cluster_id);   
    /*
     * Iterate over all clusters .... if cluster_vector_position is set
     * also the clusters with a lower have to be checked if the frontier
     * determination fails at clusters.at(cluster_vector_position). therefore
     * a ring-buffer is operated to iterate also over lower clusters, since
     * they might have changed.
     */
    int nothing_found_in_actual_cluster = 0;
    int visited_clusters = 0;

    //                    ROS_INFO("position: %lu", (cluster_vector_position +count) % (clusters.size()));
    int position = (cluster_vector_position +count) % (clusters.size());
    for (int j = 0; j < clusters.at(position).cluster_element.size(); j++)
    {     
      if(count >= clusters.size())
      {
        break;
      }

      if(clusters.at(position).cluster_element.size() > 0)
      {
        if (check_efficiency_of_goal(clusters.at(position).cluster_element.at(j).x_coordinate, clusters.at(position).cluster_element.at(j).y_coordinate) == true)
        {
          ROS_INFO("------------------------------------------------------------------");
          ROS_INFO("Robot %d determined Goal: %d  at Clusters: %d",robot_name, (int)clusters.at(position).cluster_element.at(j).id, (int)clusters.at(position).id);

          final_goal->push_back(clusters.at(position).cluster_element.at(j).x_coordinate);
          final_goal->push_back(clusters.at(position).cluster_element.at(j).y_coordinate);
          final_goal->push_back(clusters.at(position).cluster_element.at(j).detected_by_robot);
          final_goal->push_back(clusters.at(position).cluster_element.at(j).id);

          // number of the cluster we operate in
          final_goal->push_back(clusters.at(position).id);
          robot_str_name->push_back(clusters.at(position).cluster_element.at(j).detected_by_robot_str);
          return true;
        }
      }

    }
    return false;                                
  }
  return false;
}

bool sortCluster(const ExplorationPlanner::cluster_t &lhs, const ExplorationPlanner::cluster_t &rhs)
{   
  if(lhs.cluster_element.size() > 1 && rhs.cluster_element.size() > 1)
  {
    return lhs.cluster_element.front().dist_to_robot < rhs.cluster_element.front().dist_to_robot; 
  }
}

void ExplorationPlanner::sort(int strategy)
{
  //    ROS_INFO("Sorting");
  /*
   * Following Sort algorithm normalizes all distances to the 
   * robots actual position. As result, the list is sorted 
   * from smallest to biggest deviation between goal point and
   * robot! 
   */
  if(strategy == 1)
  {
    tf::Stamped < tf::Pose > robotPose;
    if (!costmap_ros_->getRobotPose(robotPose))
    {
      ROS_ERROR("Failed to get RobotPose");
    }

    if (frontiers.size() > 0) 
    {
      for (int i = frontiers.size(); i >= 0; i--) {
        for (int j = 0; j < frontiers.size() - 1; j++) {
          double x = frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
          double y = frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
          double x_next = frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
          double y_next = frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();
          double euclidean_distance = x * x + y * y;
          double euclidean_distance_next = x_next * x_next + y_next * y_next;

          if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next)) {
            frontier_t temp = frontiers.at(j+1);                                            
            frontiers.at(j + 1) = frontiers.at(j);                                          
            frontiers.at(j) = temp;                  
          }
        }
      }
    } else 
    {
      ROS_INFO("Sorting not possible, no frontiers available!!!");
    }
  }
  else if(strategy == 2)
  {
    tf::Stamped < tf::Pose > robotPose;
    if (!costmap_ros_->getRobotPose(robotPose))
    {
      ROS_ERROR("Failed to get RobotPose");
    }

    if (frontiers.size() > 0) 
    {
      for (int i = frontiers.size(); i >= 0; i--) 
      {
        for (int j = 0; j < frontiers.size() - 1; j++) {
          double x = frontiers.at(j).x_coordinate - robotPose.getOrigin().getX();
          double y = frontiers.at(j).y_coordinate - robotPose.getOrigin().getY();
          double x_next = frontiers.at(j+1).x_coordinate - robotPose.getOrigin().getX();
          double y_next = frontiers.at(j+1).y_coordinate - robotPose.getOrigin().getY();
          double euclidean_distance = x * x + y * y;
          double euclidean_distance_next = x_next * x_next + y_next * y_next;

          if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next)) {
            frontier_t temp = frontiers.at(j+1);                                            
            frontiers.at(j + 1) = frontiers.at(j);                                          
            frontiers.at(j) = temp;                  
          }
        }
      }
    }else 
    {
      ROS_INFO("Sorting not possible, no frontiers available!!!");
    }
  }
  else if(strategy == 3)
  {
    tf::Stamped < tf::Pose > robotPose;
    if (!costmap_ros_->getRobotPose(robotPose))
    {
      ROS_ERROR("Failed to get RobotPose");
    }

    if (frontiers.size() > 0) 
    {  
      check_trajectory_plan();

      for (int i = frontiers.size(); i >= frontiers.size()-10; i--) 
      {


        if(frontiers.size()-i >= frontiers.size())
        {
          break;
        }
        else
        {
          for (int j = 0; j < 10-1; j++) 
          {
            if(j >= frontiers.size())
            {
              break;
            }else
            {

              int dist = frontiers.at(j).distance_to_robot;                                   
              int dist_next = frontiers.at(j+1).distance_to_robot;

              if (dist > dist_next) {                                          
                frontier_t temp = frontiers.at(j+1);                                            
                frontiers.at(j + 1) = frontiers.at(j);                                          
                frontiers.at(j) = temp;                                          
              }
            }
          }
        }
      }
    } else 
    {
      ROS_INFO("Sorting not possible, no frontiers available!!!");
    }
  }
  else if(strategy == 4)
  {
    ROS_INFO("sort(4)");
    tf::Stamped < tf::Pose > robotPose;
    if (!costmap_ros_->getRobotPose(robotPose))
    {
      ROS_ERROR("Failed to get RobotPose");
    }
    double pose_x = robotPose.getOrigin().getX();
    double pose_y = robotPose.getOrigin().getY();
    if(clusters.size() > 0)
    {
      for(int cluster_number = 0; cluster_number < clusters.size(); cluster_number++)
      {
        if (clusters.at(cluster_number).cluster_element.size() > 0) 
        {
          ROS_DEBUG("Cluster %d  size: %lu",cluster_number, clusters.at(cluster_number).cluster_element.size());
          for (int i = clusters.at(cluster_number).cluster_element.size(); i > 0; i--) 
          {
            ROS_DEBUG("Cluster element size: %d", i);
            for (int j = 0; j < clusters.at(cluster_number).cluster_element.size()-1; j++) 
            {
              ROS_DEBUG("Cluster element number: %d", j);
              double x = clusters.at(cluster_number).cluster_element.at(j).x_coordinate - pose_x;
              double y = clusters.at(cluster_number).cluster_element.at(j).y_coordinate - pose_y;
              double x_next = clusters.at(cluster_number).cluster_element.at(j+1).x_coordinate - pose_x;
              double y_next = clusters.at(cluster_number).cluster_element.at(j+1).y_coordinate - pose_y;
              double euclidean_distance = x * x + y * y;
              double euclidean_distance_next = x_next * x_next + y_next * y_next;

              if (sqrt(euclidean_distance) > sqrt(euclidean_distance_next)) 
              {
                frontier_t temp = clusters.at(cluster_number).cluster_element.at(j+1);                                            
                clusters.at(cluster_number).cluster_element.at(j+1) = clusters.at(cluster_number).cluster_element.at(j);                                          
                clusters.at(cluster_number).cluster_element.at(j) = temp;                  
              }
            }
          }
        }
      }
    }else
    {
      ROS_INFO("Sorting not possible, no clusters available!!!");
    }
  }        
  else if(strategy == 5)
  {
    ROS_INFO("sort(5)");
    tf::Stamped < tf::Pose > robotPose;
    if (!costmap_ros_->getRobotPose(robotPose))
    {
      ROS_ERROR("Failed to get RobotPose");
    }
    double pose_x = robotPose.getOrigin().getX();
    double pose_y = robotPose.getOrigin().getY();

    ROS_DEBUG("Iterating over all cluster elements");

    for(int i = 0; i < clusters.size(); i++)
    {
      for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
      {
        clusters.at(i).cluster_element.at(j).dist_to_robot = 0;
      }
    }

    for(int i = 0; i < clusters.size(); i++)
    {
      if(clusters.at(i).cluster_element.size() > 0)
      {
        for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
        {
          /* ********** EUCLIDEAN DISTANCE ********** */
          double x = clusters.at(i).cluster_element.at(j).x_coordinate - pose_x;
          double y = clusters.at(i).cluster_element.at(j).y_coordinate - pose_y;
          double euclidean_distance = x * x + y * y;
          int distance = euclidean_distance;

          clusters.at(i).cluster_element.at(j).dist_to_robot = sqrt(distance);
        }
      }else
      {
        ROS_DEBUG("Erasing Cluster: %d", clusters.at(i).id);
        cluster_mutex.lock();
        clusters.erase(clusters.begin() + i);
        cluster_mutex.unlock();
        if(i > 0)
        {
          i --;
        }
      }
    }
    ROS_INFO("Starting to sort the clusters itself");
    std::sort(clusters.begin(), clusters.end(), sortCluster);

  }
  else if(strategy == 6)
  {
    ROS_INFO("sort(6)");
    tf::Stamped < tf::Pose > robotPose;
    if (!costmap_ros_->getRobotPose(robotPose))
    {
      ROS_ERROR("Failed to get RobotPose");
    }
    double pose_x = robotPose.getOrigin().getX();
    double pose_y = robotPose.getOrigin().getY();

    ROS_DEBUG("Iterating over all cluster elements");

    for(int i = 0; i < clusters.size(); i++)
    {
      if(clusters.at(i).cluster_element.size() > 0)
      {
        for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
        {  
          random_value = int(rand() % 100);
          clusters.at(i).cluster_element.at(j).dist_to_robot = random_value;
        }
      }else
      {
        ROS_DEBUG("Erasing Cluster: %d", clusters.at(i).id);
        clusters.erase(clusters.begin() + i);
        if(i > 0)
        {
          i --;
        }
      }
    }
    ROS_DEBUG("Starting to sort the clusters itself");
    std::sort(clusters.begin(), clusters.end(), sortCluster);  
  }  

  ROS_INFO("Done sorting");
}

void ExplorationPlanner::simulate() {

  geometry_msgs::PointStamped goalPoint;


  tf::Stamped < tf::Pose > robotPose;
  if (!costmap_ros_->getRobotPose(robotPose))
  {
    ROS_ERROR("Failed to get RobotPose");
  }
  // Visualize in RVIZ

  for (int i = frontiers.size() - 1; i >= 0; i--) {
    goalPoint.header.seq = i + 1;
    goalPoint.header.stamp = ros::Time::now();
    goalPoint.header.frame_id = "map";
    goalPoint.point.x = frontiers.at(i).x_coordinate;
    goalPoint.point.y = frontiers.at(i).y_coordinate;

    pub_Point.publish < geometry_msgs::PointStamped > (goalPoint);

    ros::Duration(1.0).sleep();
  }
}

void ExplorationPlanner::visualize_Frontiers()
{
  visualization_msgs::MarkerArray markerArray;

  for (int i = 0; i < frontiers.size(); i++)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = move_base_frame;    
    marker.header.stamp = ros::Time::now();
    marker.header.seq = frontier_seq_number++;
    marker.ns = "my_namespace";
    marker.id = frontier_seq_number;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = frontiers.at(i).x_coordinate;
    marker.pose.position.y = frontiers.at(i).y_coordinate;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;               

    markerArray.markers.push_back(marker);              
  }

  pub_frontiers_points.publish <visualization_msgs::MarkerArray>(markerArray);
}

void ExplorationPlanner::setupMapData() {

  if ((this->map_width_ != costmap_ros_->getCostmap()->getSizeInCellsX())
      || (this->map_height_ != costmap_ros_->getCostmap()->getSizeInCellsY())) {
    this->deleteMapData();

    map_width_ = costmap_ros_->getCostmap()->getSizeInCellsX();
    map_height_ = costmap_ros_->getCostmap()->getSizeInCellsY();
    num_map_cells_ = map_width_ * map_height_;

    ROS_INFO("Costmap size in cells: width:%d   height: %d   map_cells:%d",map_width_,map_height_,num_map_cells_);

  }

  occupancy_grid_array_ = costmap_ros_->getCostmap()->getCharMap();

  for (unsigned int i = 0; i < num_map_cells_; i++) {
    countCostMapBlocks(i);
  }

  ROS_INFO("--------------------- Iterate through Costmap --------------------");
  ROS_INFO("Free: %d  Inflated: %d  Lethal: %d  unknown: %d rest: %d", free,
      inflated, lethal, unknown,
      num_map_cells_ - (free + inflated + lethal + unknown));
  ROS_INFO("------------------------------------------------------------------");
  free_space = free;
}

void ExplorationPlanner::deleteMapData() {
  if (exploration_trans_array_) {
    delete[] exploration_trans_array_;
    exploration_trans_array_ = 0;
  }
  if (obstacle_trans_array_) {
    delete[] obstacle_trans_array_;
    obstacle_trans_array_ = 0;
  }
  if (is_goal_array_) {
    delete[] is_goal_array_;
    is_goal_array_ = 0;
  }
  if (frontier_map_array_) {
    delete[] frontier_map_array_;
    frontier_map_array_ = 0;
  }
}

void ExplorationPlanner::resetMaps() {
  std::fill_n(exploration_trans_array_, num_map_cells_, INT_MAX);
  std::fill_n(obstacle_trans_array_, num_map_cells_, INT_MAX);
  std::fill_n(is_goal_array_, num_map_cells_, false);
}

bool ExplorationPlanner::isSameFrontier(int frontier_point1,
    int frontier_point2) {
  unsigned int fx1, fy1;
  unsigned int fx2, fy2;
  double wfx1, wfy1;
  double wfx2, wfy2;
  costmap_.indexToCells(frontier_point1, fx1, fy1);
  costmap_.indexToCells(frontier_point2, fx2, fy2);
  costmap_.mapToWorld(fx1, fy1, wfx1, wfy1);
  costmap_.mapToWorld(fx2, fy2, wfx2, wfy2);

  double dx = wfx1 - wfx2;
  double dy = wfy1 - wfy2;

  if ((dx * dx) + (dy * dy)
      < (p_same_frontier_dist_ * p_same_frontier_dist_)) {
    return true;
  }
  return false;
}


/**
 * returns the point if it is a frontier, 0 otherwise
 */
int ExplorationPlanner::isFrontier(int point) {
  if (point > 0) {
    // number of nearby cells with no info
    int no_inf_count = 0;
    int adjacent_points[8];

    if ((int) occupancy_grid_array_[point] == costmap_2d::FREE_SPACE)
    {
      getAdjacentPoints(point, adjacent_points);
      for (int i = 0; i < 8; i++)
      {
        if (occupancy_grid_array_[adjacent_points[i]] == costmap_2d::NO_INFORMATION) 
        {
          no_inf_count++;	
        } 
        else if (occupancy_grid_array_[adjacent_points[i]] == costmap_2d::LETHAL_OBSTACLE) {
          return(0);
        }
      }
      if (no_inf_count > 0)
      {
        return(point);
      }
    }
  }
  return(0);
}

bool ExplorationPlanner::isFrontierReached(int point) {
  tf::Stamped < tf::Pose > robotPose;
  if (!costmap_ros_->getRobotPose(robotPose)) {
    ROS_WARN("[isFrontierReached]: Failed to get RobotPose");
  }
  geometry_msgs::PoseStamped robotPoseMsg;
  tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

  unsigned int fx, fy;
  double wfx, wfy;
  costmap_.indexToCells(point, fx, fy);
  costmap_.mapToWorld(fx, fy, wfx, wfy);

  double dx = robotPoseMsg.pose.position.x - wfx;
  double dy = robotPoseMsg.pose.position.y - wfy;

  if ((dx * dx) + (dy * dy)
      < (p_dist_for_goal_reached_ * p_dist_for_goal_reached_)) {
    return true;
  }
  return false;
}

inline void ExplorationPlanner::getAdjacentPoints(int point, int points[]) {
  points[0] = point + 1;
  points[1] = point - 1;
  points[2] = point + map_width_;
  points[3] = point + map_width_ + 1;
  points[4] = point + map_width_ - 1;
  points[5] = point - map_width_;
  points[6] = point - map_width_ + 1;
  points[7] = point - map_width_ - 1;
  for (int i = 0; i < 8; i++)
  {
    if (points[i] < 0 || points[i] > map_width_ * map_height_)
    {
      points[i] = -1;
    }
  }
}

bool ExplorationPlanner::countCostMapBlocks(int point) {	

  if (occupancy_grid_array_[point] == costmap_2d::NO_INFORMATION) {
    unknown++;
    return true;
  } else if ((int) occupancy_grid_array_[point] == costmap_2d::FREE_SPACE) {
    free++;
    return true;
  }

  else if ((int) occupancy_grid_array_[point] == costmap_2d::LETHAL_OBSTACLE) {
    lethal++;
    return true;
  } else if ((int) occupancy_grid_array_[point] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    inflated++;
    return true;
  } else 
  {
    int undefined = (unsigned char) occupancy_grid_array_[point];
    return true;
  }
  return false;
}

void ExplorationPlanner::clearFrontiers() {
  std::fill_n(frontier_map_array_, num_map_cells_, 0);
}
