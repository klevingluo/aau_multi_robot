#include "ExplorationPlanner.h"
#include "boost_matrix.h"
#include "hungarian.h"
#include "munkres.h"
#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
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

#define MAX_NEIGHBOR_DIST 0.30	    // radius (in cells) around selected goal without obstacles

using namespace explorationPlanner;

template <typename T>
std::string NumberToString ( T Number ) {
  std::ostringstream ss;
  ss << Number;
  return ss.str();
}

ExplorationPlanner::ExplorationPlanner(int robot_id, bool robot_prefix_empty, std::string robot_name_parameter):
  costmap_ros_(0), 
  inflated(0), 
  lethal(0), 
  free(0), 
  frontier_id_count(0), 
  exploration_travel_path_global(0), 
  cluster_id(0), 
  initialized_planner(false), 
  auction_start(false), 
  auction_finished(true), 
  auction_id_number(1), 
  next_auction_position_x(0), 
  next_auction_position_y(0), 
  other_robots_position_x(0), 
  other_robots_position_y(0),
  number_of_completed_auctions(0), 
  number_of_uncompleted_auctions(0), 
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

  std::string robo_name = "/robot_" + robot_number.str();

  std::string sendFrontier_msgs = robo_name +"/adhoc_communication/send_frontier";
  std::string sendAuction_msgs  = robo_name +"/adhoc_communication/send_auction";

  ros::NodeHandle tmp;
  nh_service = &tmp;

  ROS_DEBUG("Sending frontier: '%s'     SendAuction: '%s'", sendFrontier_msgs.c_str(), sendAuction_msgs.c_str());

  ssendFrontier = nh_service->serviceClient<adhoc_communication::SendExpFrontier>(sendFrontier_msgs);
  ssendAuction = nh_service->serviceClient<adhoc_communication::SendExpAuction>(sendAuction_msgs);

  pub_clusters = nh.advertise <visualization_msgs::MarkerArray> ("clusters", 200, true);

  pub_Point = nh_Point.advertise < geometry_msgs::PointStamped> ("goalPoint", 100, true);
  pub_frontiers_points = nh_frontiers_points.advertise <visualization_msgs::MarkerArray> ("frontierPoints", 2000, true);

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

/**
 * sort fronters from lst to clusters
 */
bool ExplorationPlanner::clusterFrontiers() {
  ROS_INFO("clusterFrontiers");
  clusters.clear();

  for(int i = 0; i < frontiers.size(); i++)
  {
    // clusters that this frontier belongs to
    vector<int> cluster_indices;
    for(int j = 0; j < clusters.size(); j++)
    {
      for(int n = 0; n < clusters.at(j).cluster_element.size(); n++)
      {
        if(fabs(frontiers.at(i).x_coordinate - clusters.at(j).cluster_element.at(n).x_coordinate) < MAX_NEIGHBOR_DIST && 
            fabs(frontiers.at(i).y_coordinate - clusters.at(j).cluster_element.at(n).y_coordinate) < MAX_NEIGHBOR_DIST) 
        {
          cluster_indices.push_back(j);
          break;
        }
      }
    }
    if (cluster_indices.size() > 0)
    {
      std::vector<frontier_t>& main_clus = clusters.at(0).cluster_element;
      main_clus.push_back(frontiers.at(i));
      for (int j = cluster_indices.size() - 1; j > 0; j--)
      {
        std::vector<frontier_t>& sub_clus = clusters.at(cluster_indices.at(j)).cluster_element;
        main_clus.insert(main_clus.end(), sub_clus.begin(), sub_clus.end());
        clusters.erase(clusters.begin() + cluster_indices.at(j));
      }
    } else
    {
      cluster_t cluster_new;
      cluster_new.cluster_element.push_back(frontiers.at(i));
      cluster_new.id = (robot_name * 10000) + cluster_id++;
      clusters.push_back(cluster_new); 
    }
  }
  ROS_INFO("found %lu clusters", clusters.size());
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
      visualization_msgs::Marker marker = createMarker();

      marker.color.r = color_r;
      marker.color.g = color_g;
      marker.color.b = color_b;               
      marker.pose.position.x = clusters.at(j).cluster_element.at(n).x_coordinate;
      marker.pose.position.y = clusters.at(j).cluster_element.at(n).y_coordinate;

      clustersArray.markers.push_back(marker);              
    }           
  }
  pub_clusters.publish <visualization_msgs::MarkerArray>(clustersArray);
}

bool ExplorationPlanner::transformToOwnCoordinates_frontiers() { 
  ROS_INFO("Transform frontier coordinates");

  store_frontier_mutex.lock();

  for(int i = 0; i < frontiers.size(); i++) {        
    if(frontiers.at(i).detected_by_robot != robot_name)
    {
      bool transform_flag = false;
      for(int j=0; j < transformedPointsFromOtherRobot_frontiers.size(); j++)
      {
        if(robot_prefix_empty_param == 0)
        {
          if(transformedPointsFromOtherRobot_frontiers.at(j).id == frontiers.at(i).id && frontiers.at(i).detected_by_robot_str.compare(transformedPointsFromOtherRobot_frontiers.at(j).robot_str)== 0)
          {
            transform_flag = true;
            break;
          }
        } else {
          if(transformedPointsFromOtherRobot_frontiers.at(j).id == frontiers.at(i).id)
          {
            transform_flag = true;
            break;
          }
        }     
      }

      if(transform_flag != true)
      {
        std::string robo_name, robo_name2;

        if(robot_prefix_empty_param == false)
        {
          std::stringstream robot_number;
          robot_number << frontiers.at(i).detected_by_robot;

          std::string prefix = "robot_";
          robo_name = prefix.append(robot_number.str());                
          ROS_DEBUG("Robots name is        %s", robo_name.c_str());

          std::stringstream robot_number2;
          robot_number2 << robot_name;

          std::string prefix2 = "/robot_";
          robo_name2 = prefix2.append(robot_number2.str());
        }
        else
        {      
          robo_name = frontiers.at(i).detected_by_robot_str;
          robo_name2 = robot_str;
          ROS_DEBUG("Robot: %s   transforms from robot: %s", robo_name2.c_str(), robo_name.c_str());
        }



        std::string service_topic = robo_name2.append("/map_merger/transformPoint"); // FIXME for real scenario!!! robot_might not be used here

        client = nh_transform.serviceClient<map_merger::TransformPoint>(service_topic);

        service_message.request.point.x = frontiers.at(i).x_coordinate;
        service_message.request.point.y = frontiers.at(i).y_coordinate;
        service_message.request.point.src_robot = robo_name;

        ROS_DEBUG("Robot name:  %s", service_message.request.point.src_robot.c_str());
        ROS_DEBUG("Old x: %f   y: %f", frontiers.at(i).x_coordinate, frontiers.at(i).y_coordinate);

        if(client.call(service_message))
        {
          frontiers.at(i).x_coordinate = service_message.response.point.x; 
          frontiers.at(i).y_coordinate = service_message.response.point.y;

          transform_point_t transform_point;
          transform_point.id = frontiers.at(i).id;

          if(robot_prefix_empty_param == true)
          {
            transform_point.robot_str = frontiers.at(i).detected_by_robot_str;
          }
          transformedPointsFromOtherRobot_frontiers.push_back(transform_point);
          ROS_DEBUG("New x: %.1f   y: %.1f",service_message.response.point.x, service_message.response.point.y);                   
        }
      }
    }
  }
  store_frontier_mutex.unlock();
  ROS_INFO(" Transform frontier coordinates DONE");
}

/**
 * is the plan successful?, write the distances to
 frontiers.at(i).distance_to_robot = distance;
 */
bool ExplorationPlanner::check_trajectory_plan() {       
  for(int i = 0; i<10 && i < frontiers.size(); i++) {
    int distance = check_trajectory_plan(
        frontiers.at(i).x_coordinate, 
        frontiers.at(i).y_coordinate);

    frontiers.at(i).distance_to_robot = distance;
    ROS_DEBUG("Distance: %d Frontier: %d",distance, frontiers.at(i).id);
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
      odom->pose.pose.position.x,
      odom->pose.pose.position.y,
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

  // oops, this is broken but whatever
  // bool successful = nav.makePlan(startPointSimulated, goalPointSimulated, global_plan);
  bool successful = false;

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
bool ExplorationPlanner::storeFrontier(
    int x, 
    int y, 
    int map_index,
    int value,
    int detected_by_robot, 
    std::string detected_by_robot_str, 
    int id) 
{

  frontier_t new_frontier;

  if(detected_by_robot != robot_name)
  {
    new_frontier.id = id;
  } else
  {
    new_frontier.id = (robot_name * 10000) + frontier_id_count++; 
  }

  new_frontier.detected_by_robot = detected_by_robot;
  new_frontier.x_coordinate = x * resolution_ + map_origin_.position.x + resolution_ / 2.0;       
  new_frontier.y_coordinate = y * resolution_ + map_origin_.position.y + resolution_ / 2.0;
  new_frontier.mapx = x + (int)(map_origin_.position.x);
  new_frontier.mapy = y + (int)(map_origin_.position.y);
  new_frontier.map_index = map_index;
  new_frontier.value = value;

  store_frontier_mutex.lock(); 
  frontiers.push_back(new_frontier);
  store_frontier_mutex.unlock();

  return true;
}

bool ExplorationPlanner::removeStoredFrontier(int id, std::string detected_by_robot_str) {
  for(int i= 0; i < frontiers.size(); i++)
  {
    ROS_DEBUG("Removing frontier with id '%d' detected by robot '%s'", frontiers.at(i).id, frontiers.at(i).detected_by_robot_str.c_str());
    if(frontiers.at(i).id == id)
    {
      frontiers.erase(frontiers.begin()+i);
    }
  }
  return true;
}

bool ExplorationPlanner::storeVisitedFrontier(double x, double y, int detected_by_robot, std::string detected_by_robot_str, int id)
{
  frontier_t visited_frontier;

  if(robot_prefix_empty_param == true)
  {        
    ROS_DEBUG("Storing Visited Frontier ID: %d   Robot: %s", visited_frontier_id_count, detected_by_robot_str.c_str());
    if(id != -1)
    {
      visited_frontier.id = id;
    }else
    {
      visited_frontier.id = visited_frontier_id_count++;
    }

    visited_frontier.detected_by_robot_str = detected_by_robot_str;
    visited_frontier.x_coordinate = x;
    visited_frontier.y_coordinate = y;

    store_visited_mutex.lock();
    visited_frontiers.push_back(visited_frontier);
    store_visited_mutex.unlock();

  }else
  {
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
  }

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
bool ExplorationPlanner::storeUnreachableFrontier(
    double x, 
    double y, 
    int detected_by_robot, 
    std::string detected_by_robot_str, 
    int id)
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

  unreachable_frontiers.push_back(unreachable_frontier);
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

  if(robot_prefix_empty_param == true)
  {
    robo_name = robot_str;
  }
  std::string destination_name = multi_cast_group + robo_name; //for multicast

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

  if(robot_prefix_empty_param == true)
  {
    robo_name = robot_str;
  }

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
      if(robot_prefix_empty_param == true)
      {
        if(negotiation_list.at(i).id == frontier_element.id && negotiation_list.at(i).detected_by_robot_str.compare(frontier_element.detected_by_robot_str) == 0)
        {
          entry_found = true;       
        }  
      }else
      {
        if(negotiation_list.at(i).id == frontier_element.id)
        {
          entry_found = true;       
        }  
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
          if(robot_prefix_empty_param == true)
          {
            cluster_element_msg.id = requested_cluster_ids.at(n).requested_ids.at(j).id;
            cluster_element_msg.detected_by_robot_str = requested_cluster_ids.at(n).requested_ids.at(j).robot_str;
          } else
          {
            cluster_element_msg.id = requested_cluster_ids.at(n).requested_ids.at(j).id;
          }
          cluster_msg.ids_contained.push_back(cluster_element_msg);
        }

        cluster_msg.bid = calculateAuctionBID(
            clusters.at(i).id, 
            trajectory_strategy,
            odom->pose.pose.position.x,
            odom->pose.pose.position.y);
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
        if (distance * costmap_ros_->info.resolution <= sqrt(euclidean_distance)*0.95) 
        {
          ROS_WARN("Euclidean distance smaller then trajectory distance to LOCAL CLUSTER!!!");
          return(-1);
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
  bool same_robot = false; 

  if(robot_prefix_empty_param == true)
  {
    if(robot_str.compare(msg.get()->robot_name.c_str()) == 0)
    {
      same_robot = true; 
    }
  } else
  {
    robots_int_name = atoi(msg.get()->robot_name.c_str());
    if(robots_int_name == robot_name)
    {
      same_robot = true; 
    }
  }

  if(same_robot == false)
  {
    /*
     * Cluster all available frontiers to be able to compare clusters
     * with other robots
     */

    /*
     * TODO
     * Check if following code is necessary or if simple navigation needs to 
     * periodically update the frontiers in the frontier thread.  
     */
    transformToOwnCoordinates_frontiers();
    clearUnreachableFrontiers();
    clusterFrontiers();

    if(msg.get()->auction_status_message == true)
    { 
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
        } else
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
              if(robot_prefix_empty_param == true)
              {
                cluster_element_point.id = requested_cluster.ids_contained.at(j).id;
                cluster_element_point.robot_str = requested_cluster.ids_contained.at(j).detected_by_robot_str;                    
              }else
              {
                cluster_element_point.id = requested_cluster.ids_contained.at(j).id;                                                          
              }

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

    }else if(msg.get()->auction_status_message == false)
    {
      bool continue_auction = false; 
      if(robot_prefix_empty_param == true)
      {
        if(msg.get()->auction_id == auction_id_number)
        {
          continue_auction = true; 
        }
      }else
      {
        if(msg.get()->auction_id == 10000*robot_name + auction_id_number)
        {
          continue_auction = true; 
        }
      }

      if(continue_auction == true)
      {
        //            ROS_INFO("auction_id: %d       local_id: %d", msg.get()->auction_id, 10000*robot_name + auction_id_number);
        bool robot_already_answered = false; 

        for(int i = 0; i < robots_already_responded.size(); i++)
        {
          if(robot_prefix_empty_param == true)
          {
            if(msg.get()->robot_name.compare(robots_already_responded.at(i).robot_str) == 0 && msg.get()->auction_id == robots_already_responded.at(i).auction_number)
            {
              ROS_WARN("Same msg already received!!!");
              robot_already_answered = true; 
              break;
            }
          }else
          {
            ROS_INFO("Compare msg name: %d  responded: %d       msg auction: %d   responded: %d", robots_int_name, robots_already_responded.at(i).robot_number, msg.get()->auction_id, robots_already_responded.at(i).auction_number);
            if(robots_int_name == robots_already_responded.at(i).robot_number && msg.get()->auction_id == robots_already_responded.at(i).auction_number)
            {
              ROS_WARN("Same msg already received!!!");
              robot_already_answered = true; 
              break;
            }
          }
        }

        /*
         * Only proceed if the robot has not answered before. 
         */
        if(robot_already_answered == false)
        {
          if(robot_prefix_empty_param == true)
          {
            ROS_INFO("Auction from robot %s received", msg.get()->robot_name.c_str());
          }else
          {
            ROS_INFO("Auction from robot %d received", robot_name);          
          }
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
          //                {
          number_of_auction_bids_received++;
          responded_t auction_response; 
          auction_response.auction_number = msg.get()->auction_id;
          auction_response.robot_number = robots_int_name;
          if(robot_prefix_empty_param == true)
          {
            auction_response.robot_str = msg.get()->robot_name;
          }
          robots_already_responded.push_back(auction_response);
          //                }
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
        if(robot_prefix_empty_param == true)
        {
          if(clusters.at(j).cluster_element.at(n).id == cluster_to_check.ids_contained.at(i).id && clusters.at(j).cluster_element.at(n).detected_by_robot_str.compare(cluster_to_check.ids_contained.at(i).detected_by_robot_str) == 0)
          {
            same_id_found++;
          }
        }else
        {
          if(clusters.at(j).cluster_element.at(n).id == cluster_to_check.ids_contained.at(i).id)
          {
            same_id_found++;
          }
        }
      }                  
    }
    if(same_id_found > clusters.at(j).cluster_element.size() * 0.4)
    {
      return (clusters.at(j).id);
    } 
    return (-1);
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
      if(robot_prefix_empty_param == true)
      {
        ROS_DEBUG("Storing Visited Frontier ID: %ld  Robot: %s", frontier_element.id, frontier_element.detected_by_robot_str.c_str());
        storeVisitedFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, frontier_element.detected_by_robot_str, frontier_element.id);
      }else
      {
        if(frontier_element.detected_by_robot != robot_name)
        {    
          storeVisitedFrontier(frontier_element.x_coordinate, frontier_element.y_coordinate, frontier_element.detected_by_robot, "", frontier_element.id); 
        }  
      }
    } 
  }
}

void ExplorationPlanner::clearUnreachableFrontiers()
{
  std::vector<int> goals_to_clear;

  for (int i = 1; i < unreachable_frontiers.size(); i++)
  {
    for (int j = 0; j < frontiers.size(); j++)
    {
      double diff_x = unreachable_frontiers.at(i).x_coordinate - frontiers.at(j).x_coordinate;
      double diff_y = unreachable_frontiers.at(i).y_coordinate - frontiers.at(j).y_coordinate;

      if (fabs(diff_x) <= 0.2 && fabs(diff_y) <= 0.2) {
        if(robot_prefix_empty_param == true)
        {
          removeStoredFrontier(frontiers.at(j).id, frontiers.at(j).detected_by_robot_str);
          if(j > 0)
          {
            j--;
          }
        }else
        {
          removeStoredFrontier(frontiers.at(j).id, "");
          if(j > 0)
          {
            j--;
          }
        }
        break;
      }
    }
  }
}

/*
 * searches the occupancy grid for frontier cells and merges them into one target point per frontier.
 * The returned frontiers are in world coordinates.
 */
void ExplorationPlanner::findFrontiers() {
  ROS_INFO("called find frontiers: %i cells", num_map_cells_); 
  allFrontiers.clear();
  frontiers.clear();

  for (unsigned int i = 0; i < num_map_cells_; i++) {
    int new_frontier_point = isFrontier(i);
    if (new_frontier_point != 0) {
      allFrontiers.push_back(new_frontier_point);
    }
  }
  ROS_INFO("Found %lu frontier cells which are transformed into frontiers points. Starting transformation...", allFrontiers.size());

  for (unsigned int i = 0; i < allFrontiers.size(); ++i) {
    storeFrontier(
        allFrontiers.at(i) % map_width_,
        allFrontiers.at(i) / map_width_,
        allFrontiers.at(i),
        0,
        robot_name,
        robot_str,
        -1);
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


  //    adhoc_communication::AuctionStatus auction_status;
  adhoc_communication::ExpAuction reqest_clusters, auction_msg;
  //    auction_status.start_auction = true; 
  //    auction_status.auction_finished = false;

  auction_msg.start_auction = true; 
  auction_msg.auction_finished = false;
  auction_msg.auction_status_message = true;

  if(robot_prefix_empty_param == false)
  {
    std::stringstream ss;
    ss << robot_name; 
    std::string prefix = "";
    robo_name = prefix.append(ss.str());    

    auction_msg.robot_name = robo_name;
  }else
  {
    auction_msg.robot_name = robot_str;
    robo_name = robot_str;
  }

  for(int i = 0; i < clusters.size(); i++)
  {
    adhoc_communication::ExpCluster cluster_request;


    for(int j = 0; j < clusters.at(i).cluster_element.size(); j++)
    { 
      adhoc_communication::ExpClusterElement cluster_request_element;
      if(robot_prefix_empty_param == true)
      {
        cluster_request_element.id = clusters.at(i).cluster_element.at(j).id;
        cluster_request_element.detected_by_robot_str = clusters.at(i).cluster_element.at(j).detected_by_robot_str;
      }else
      {
        cluster_request_element.id = clusters.at(i).cluster_element.at(j).id;
      }
      cluster_request.ids_contained.push_back(cluster_request_element);
    }
    auction_msg.requested_clusters.push_back(cluster_request);
  }

  if(robot_prefix_empty_param == true)
  {
    auction_msg.auction_id = auction_id_number;
  }else
  {
    auction_msg.auction_id = 10000*robot_name + auction_id_number;
  }

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
        odom->pose.pose.position.x,
        odom->pose.pose.position.y);
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

            if(robot_prefix_empty_param == true)
            {
              ROS_INFO("position of robot: %s   compare with auction robot: %s", position_of_robot.src_robot.c_str(), auction.at(i).detected_by_robot_str.c_str());
              if(position_of_robot.src_robot.compare(auction.at(i).detected_by_robot_str) == 0)
              {
                other_robots_position_x = position_of_robot.x;
                other_robots_position_y = position_of_robot.y;

                break; 
              }
              else
              {
                ROS_ERROR("Unable to look up robots position!");
              }
            }else
            {
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
  if(strategy == 2)
  {
    final_goal->push_back(frontiers.at(0).x_coordinate);
    final_goal->push_back(frontiers.at(0).y_coordinate);
    final_goal->push_back(frontiers.at(0).detected_by_robot);
    final_goal->push_back(frontiers.at(0).id);

    robot_str_name->push_back(frontiers.at(0).detected_by_robot_str);
    return true;
  }    
  else if(strategy == 5)
  {
    final_goal->push_back(clusters.at(0).cluster_element.at(0).x_coordinate);
    final_goal->push_back(clusters.at(0).cluster_element.at(0).y_coordinate);
    final_goal->push_back(clusters.at(0).cluster_element.at(0).detected_by_robot);
    final_goal->push_back(clusters.at(0).cluster_element.at(0).id);

    robot_str_name->push_back(clusters.at(0).cluster_element.at(0).detected_by_robot_str);
    return true;
  }
  else if(strategy == 6)
  {
    cv::Mat im(map_height_, map_width_, CV_8UC1);

    for (int i=0; i < map_width_ * map_height_; i++)
    {
      im.data[i] = 0;
    }
    for (int i = 0; i < frontiers.size(); i++)
    {
      im.data[frontiers.at(i).map_index] = 255;
    }

    cv::Mat kernel = (cv::getGaussianKernel(39, 0) * -1 + 0.01);
    cv::normalize(kernel, kernel);

    filter2D(im, im, -1 , kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT );

    frontiers.clear();
    for (int i=0; i < map_width_ * map_height_; i++)
    {
      int adjacent_points[8];
      getAdjacentPoints(i, adjacent_points);

      bool flag = false;
      for (int j = 0; j < 8; j++)
      {
        if (adjacent_points[j] >= 0 && 
            occupancy_grid_array_[adjacent_points[j]] > 50) 
        {
          flag = true;
        }

      }

      if (flag)
        continue;

      if (im.data[i] > 10.33 && 
          occupancy_grid_array_.at(i) != -1 && 
          occupancy_grid_array_.at(i) < 100 &&
          i % 5 == 1
         )
      {
        storeFrontier(
            i % map_width_,
            i / map_width_,
            i,
            im.data[i],
            robot_name,
            robot_str,
            -1);
      }
    }
    clearUnreachableFrontiers();
    visualize_Frontiers();

    sort(1);

    final_goal->push_back(frontiers.at(0).x_coordinate);
    final_goal->push_back(frontiers.at(0).y_coordinate);
    final_goal->push_back(frontiers.at(0).detected_by_robot);
    final_goal->push_back(frontiers.at(0).id);
    robot_str_name->push_back(frontiers.at(0).detected_by_robot_str);
    return true;
  }
  return false;
}

float euclidDist(float x1, float x2, float y1, float y2)
{
  float xDist = x2 - x1;
  float yDist = y2 - y1;
  return xDist*xDist + yDist*yDist;
}

void ExplorationPlanner::sort(int strategy)
{

  double xCoor = odom->pose.pose.position.x;
  double yCoor = odom->pose.pose.position.y;

  if(strategy == 1 || strategy == 2)
  {
    std::sort(
        frontiers.begin(), 
        frontiers.end(), 
        [xCoor, yCoor] (frontier_t front1, frontier_t front2) {
        return euclidDist(xCoor, front1.x_coordinate, yCoor, front1.y_coordinate) < 
        euclidDist(xCoor, front2.x_coordinate, yCoor, front2.y_coordinate);
        });
  }
  else if(strategy == 3)
  {
    check_trajectory_plan();
    std::sort(
        frontiers.begin(), 
        frontiers.begin() + 10, 
        [] (frontier_t front1, frontier_t front2) {return front1.distance_to_robot < front2.distance_to_robot;});
  }
  else if(strategy == 4)
  {
    for(int cluster_number = 0; cluster_number < clusters.size(); cluster_number++)
    {
      std::sort(
          clusters.at(cluster_number).cluster_element.begin(), 
          clusters.at(cluster_number).cluster_element.end(), 
          [xCoor, yCoor] (frontier_t front1, frontier_t front2) {
          return euclidDist(xCoor, front1.x_coordinate, yCoor, front1.y_coordinate) < 
          euclidDist(xCoor, front2.x_coordinate, yCoor, front2.y_coordinate);
          });
    }
    std::sort(
        clusters.begin(), 
        clusters.end(), 
        [xCoor, yCoor] (cluster_t clus1, cluster_t clus2) {
        return euclidDist(xCoor, clus1.cluster_element.at(0).x_coordinate, yCoor, clus1.cluster_element.at(0).y_coordinate) < 
        euclidDist(xCoor, clus2.cluster_element.at(0).x_coordinate, yCoor, clus2.cluster_element.at(0).y_coordinate);
        });
  }
  ROS_INFO("Done sorting");
}

void ExplorationPlanner::updateLocalCostmap(nav_msgs::OccupancyGrid* cm)
{
}

void ExplorationPlanner::updateGlobalCostmap(nav_msgs::OccupancyGrid* cm)
{
  this->costmap_ros_ = cm;
  map_width_ = cm->info.width;
  map_height_ = cm->info.height;
  num_map_cells_ = map_width_ * map_height_;
  occupancy_grid_array_ = cm->data;
  map_origin_ = cm->info.origin;
  resolution_ = cm->info.resolution;
  initialized_planner = true;
}

void ExplorationPlanner::updateOdom(nav_msgs::Odometry* odom)
{
  this->odom = odom;
}

void ExplorationPlanner::simulate() {

  geometry_msgs::PointStamped goalPoint;

  for (int i = frontiers.size() - 1; i >= 0; i--) {
    goalPoint.header.seq = i + 1;
    goalPoint.header.stamp = ros::Time::now();
    goalPoint.header.frame_id = "map";
    goalPoint.point.x = frontiers.at(i).x_coordinate;
    goalPoint.point.y = frontiers.at(i).y_coordinate;

    pub_Point.publish <geometry_msgs::PointStamped> (goalPoint);

    ros::Duration(1.0).sleep();
  }
}

void ExplorationPlanner::visualize_Frontiers()
{
  // action 3: delete all markers
  visualization_msgs::Marker deleteall;
  deleteall.action = 3;

  visualization_msgs::MarkerArray deleteallArray;
  deleteallArray.markers.push_back(deleteall);              

  pub_frontiers_points.publish <visualization_msgs::MarkerArray>(deleteallArray);

  ROS_INFO("publishing %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markerArray;

  for (int i = 0; i < frontiers.size(); i++)
  {
    visualization_msgs::Marker marker = createMarker();
    marker.pose.position.x = frontiers.at(i).x_coordinate;
    marker.pose.position.y = frontiers.at(i).y_coordinate;
    marker.color.r = frontiers.at(i).value/255.0;
    markerArray.markers.push_back(marker);
  }

  pub_frontiers_points.publish <visualization_msgs::MarkerArray>(markerArray);
}

/**
 * returns the point if it is a frontier, 0 otherwise
 */
int ExplorationPlanner::isFrontier(int point) {

  // ignore invalid points, unknown points, and obstacles
  if (point <= 0 || occupancy_grid_array_[point] == -1 || occupancy_grid_array_[point] > 50)
    return 0;

  int adjacent_points[8];
  getAdjacentPoints(point, adjacent_points);

  for (int i = 0; i < 8; i++)
  {
    if (adjacent_points[i] > 0 && occupancy_grid_array_[adjacent_points[i]] == -1) 
      return(point);
  }
  return 0;
}

inline visualization_msgs::Marker ExplorationPlanner::createMarker() {
  visualization_msgs::Marker marker;

  marker.header.frame_id = move_base_frame;    
  marker.header.stamp = ros::Time::now();
  marker.header.seq = frontier_seq_number++;
  marker.ns = "marker";
  marker.id = frontier_seq_number;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
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
  marker.color.g = 1.0;
  marker.color.b = 0.0;               

  return marker;
}

/**
 * writes adjacent points to points, -1 if they are out of bounds
 */
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
    if (points[i] < 0 || points[i] > num_map_cells_)
    {
      points[i] = -1;
    }
  }
}
