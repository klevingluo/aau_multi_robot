#include <fstream>
#include <cerrno>
#include <boost/filesystem.hpp>
#include "unistd.h"
#include "mapmerger.h"
#include "adhoc_communication/MmPoint.h"

MapMerger::MapMerger()
{    
  pos_seq_my = 0;
  pos_pub_other = new std::vector<ros::Publisher>();
  pos_array_other = new std::vector<visualization_msgs::MarkerArray>();
  pos_seq_other = new std::vector<int>();

  global_map = NULL;
  local_map = NULL;
  local_map_old = NULL;
  updateMan = new updateManager();

  positions = new adhoc_communication::MmListOfPoints();
  map_data = new std::vector<nav_msgs::OccupancyGrid*>();
  robots = new std::vector<std::string>();
  transforms = new std::vector<cv::Mat>();
  new_data_maps = new std::vector<bool>();
  robots_in_transform = new std::vector<std::string>();
  robots_position_publisher = new std::vector<ros::Publisher>();
  local_map = new nav_msgs::OccupancyGrid();
  local_map_old = new nav_msgs::OccupancyGrid();
  update_list = new std::vector<UpdateEntry*>();
  laser_range = 10;
  force_recompute_all = false;
  local_map_new_data = false;
  global_map_ready = true;
  cur_position = new geometry_msgs::PoseStamped();
  map_width = -1;
  map_height = -1;
  update_seq = 0;
  nodeHandle = new ros::NodeHandle("~");
  nodeHandle->param<std::string>("robot_name",robot_name,"");
  if(robot_name.empty())
  {
    char hostname_c[1024];
    hostname_c[1023] = '\0';
    int status = gethostname(hostname_c, 1023);
    if(status == -1)
      ROS_ERROR("Cannot determine hostname. ERROR: %s", strerror(errno));
    else
      robot_name = std::string(hostname_c);
  }
  nodeHandle->param<bool>("splitted",splitted,true);;
  nodeHandle->param<bool>("debug",debug,true);
  nodeHandle->param<bool>("has_local_map",has_local_map,true);
  nodeHandle->param<bool>("exchange_position",exchange_position,false);

  nodeHandle->param<int>("split_size",size,32);
  nodeHandle->param<int>("seconds_meta_timer",seconds_meta_timer,5);
  nodeHandle->param<int>("seconds_pub_timer",seconds_publish_timer,3);
  nodeHandle->param<int>("seconds_send_timer",seconds_send_timer,5);
  nodeHandle->param<int>("seconds_recompute_transform",seconds_recompute_transform,5);
  nodeHandle->param<int>("seconds_send_position",seconds_send_position,2);
  nodeHandle->param<double>("max_trans_robots",max_trans_robot,-1);
  nodeHandle->param<double>("max_rotation_robots",max_rotation_robot,5);

  nodeHandle->param<std::string>("local_map_topic",local_map_topic,"/map");
  nodeHandle->param<std::string>("local_map_metadata_topic",local_map_metadata_topic,"/map_metadata");
  nodeHandle->param<std::string>("local_map_frame_id",local_map_frame_id,"map");
  nodeHandle->param<std::string>("map_topic_over_network",topic_over_network,"map_other");
  nodeHandle->param<std::string>("position_local_robot_topic",position_local_robot_topic,"slam_out_pose");
  nodeHandle->param<std::string>("position_other_robots_topic",position_other_robots_topic,"position_other_robots");
  nodeHandle->param<std::string>("control_topic",control_topic,"control");
  nodeHandle->param<std::string>("robot_prefix",robot_prefix,"");
  nodeHandle->param<std::string>("log_path",log_path,"");

  time_start = ros::Time::now();

  ROS_INFO("Local map topic:%s",local_map_topic.c_str());
  ROS_INFO("Local map meta data:%s",local_map_metadata_topic.c_str());
  ROS_INFO("Width:%i",map_width);
  ROS_INFO("Height:%i",map_height);
  ROS_INFO("Splitted:%i",splitted);
  ROS_INFO("Has local map:%i",has_local_map);
  ROS_INFO("Split size:%i",size);
  ROS_INFO("Robot name:%s",robot_name.c_str());
  ROS_INFO("Prefix:%s",robot_prefix.c_str());
  ROS_INFO("Local Map Frame:%s",local_map_frame_id.c_str());
  robot_hostname = robot_name;

}

void MapMerger::waitForLocalMetaData()
{
  ROS_INFO("Wait for map, on topic:[%s]",local_map_topic.c_str());
  ros::Subscriber  sub = nodeHandle->subscribe(local_map_topic,1000,&MapMerger::callback_map_meta_data_local,this);
  while(ros::ok())
  {
    ros::spinOnce();
    ROS_DEBUG("No map_meta information");
    ros::Duration(0.3).sleep();
    if(this->map_height != -1 && this->map_width != -1)
      break;
  }
  ROS_INFO("Got map_meta_data");
  if(!splitted)
  {
    size = map_height;
  }
}

void MapMerger::waitForRobotInformation()
{
  ROS_INFO("Wait to get a map from other robot");
  ros::Subscriber sub =nodeHandle->subscribe(robot_prefix+"/adhoc_communication/new_robot",
      1000,&MapMerger::callback_got_robot_for_data,

      this);


  ros::ServiceClient getNeighborsClient = nodeHandle->serviceClient<adhoc_communication::GetNeighbors>
    (robot_prefix+"/adhoc_communication/get_neighbors");
  adhoc_communication::GetNeighbors getNeighbors;
  ros::Duration(0.1).sleep();
  ros::AsyncSpinner spinner(10);
  spinner.start();
  ros::Duration(3).sleep();
  if(getNeighborsClient.call(getNeighbors))
  {
    ROS_INFO("%lu robots are already in the network, adding them,my name:%s",getNeighbors.response.neigbors.size(),
        robot_name.c_str());
    if(getNeighbors.response.neigbors.size() > 0)
    {
      for(int i = 0; i < getNeighbors.response.neigbors.size();i++)
      {
        std::string newRobotName = getNeighbors.response.neigbors.at(i);
        if(newRobotName == robot_name)
        {
          ROS_WARN("Got my own name as a neigbor!");
          continue;
        }
        else
        {
          if(has_local_map == false)
          {
            robot_name = newRobotName;
            ROS_INFO("now using %s as name",robot_name.c_str());
            return;
          }
        }
      }
    }
    else
    {
      ROS_INFO("No robots are were in the network before me");
    }
  }
  while(ros::ok())
  {
    ROS_INFO("No robot information");
    ros::spinOnce();
    ros::Duration(0.3).sleep();
    if(robot_name != "BASE")
      break;
  }
}

void MapMerger::callback_got_robot_for_data(const std_msgs::StringConstPtr &msg)
{
  ROS_INFO("Using %s as pseudo local map",msg.get()->data.c_str());
  this->robot_name = msg.get()->data;
}

void MapMerger::callback_new_robot(const std_msgs::StringConstPtr &msg)
{
  std::string newRobotName = msg.get()->data;
  std::vector<int>* empty = new std::vector<int>();
  ROS_INFO("Sending Control Messate to robot: %s", newRobotName.c_str());
  sendControlMessage(empty,newRobotName);
  delete empty;
}

void MapMerger::callback_control(const adhoc_communication::MmControlConstPtr &msg)
{
  ROS_INFO("Got a control message");
  adhoc_communication::MmControl tmp = *msg.get();
  ros::ServiceClient client = nodeHandle->serviceClient<adhoc_communication::SendOccupancyGrid>
    (robot_prefix + "/adhoc_communication/send_map");
  std::string service = robot_prefix + "/adhoc_communication/send_map";

  ROS_INFO("Sending map to: %s on topic: %s", tmp.src_robot.c_str(), topic_over_network.c_str());
  adhoc_communication::SendOccupancyGrid exchange;
  exchange.request.topic = topic_over_network;
  exchange.request.dst_robot = tmp.src_robot.c_str();
  exchange.request.map = *local_map;

  if(client.call(exchange))
  {
    if(exchange.response.status)
      ROS_DEBUG("Sent Map to:%s,topic:%s",exchange.request.dst_robot.c_str(),exchange.request.topic.c_str());
    else
      ROS_WARN("Destination host unreachable: [%s]",tmp.src_robot.c_str());
  }
  else
    ROS_DEBUG("Map sending service unavailable");

  //also send the location
  ROS_WARN("sending location over network");
  sendLocationOverNetwork(tmp.src_robot.c_str());
}

void MapMerger::callback_map_meta_data_local(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  ROS_DEBUG("Got local map meta data");
  map_width = msg.get()->info.width;
  map_height = msg.get()->info.height;
  ROS_INFO("height: %i", msg.get()->info.height);

  g_start_x = msg.get()->info.origin.position.x;
  g_start_y = msg.get()->info.origin.position.y;

  if(!has_local_map)
  {
    local_map = new nav_msgs::OccupancyGrid();
    local_map->info.height = msg.get()->info.height;
    local_map->info.width = msg.get()->info.width;
    local_map->info.origin.orientation.w = msg.get()->info.origin.orientation.w;
    local_map->info.resolution = msg.get()->info.resolution;

    global_map = new nav_msgs::OccupancyGrid();
    global_map->info.height = msg.get()->info.height;
    global_map->info.width = msg.get()->info.width;
    global_map->info.origin.orientation.w = msg.get()->info.origin.orientation.w;
    global_map->info.resolution = msg.get()->info.resolution;

    for(int i = 0; i < map_height * map_width; i++)
    {
      global_map->data.push_back(-1);
      local_map->data.push_back(-1);
    }
  }
}

void MapMerger::callback_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_DEBUG("Start callback_map");
  nav_msgs::OccupancyGrid tmp = *msg.get();
  nav_msgs::OccupancyGrid * toInsert = &tmp;
  if(toInsert->header.frame_id == local_map_frame_id)
  {
    ROS_INFO("Updating local map");
    processLocalMap(toInsert,-1);
    force_recompute_all = true;
    return;
  }
}

void MapMerger::callback_global_pub(const ros::TimerEvent &e)
{
  if(robots->size() == 0)
  {
    ROS_INFO("No other robots, publishing local map");
    if(local_map != NULL && local_map->data.size() > 0)
      pub.publish(*local_map);
    return;
  }
  if(transforms->size() != map_data->size() && local_map != NULL )
  {
    for(size_t i = 0; i < robots->size(); i++)
    {
      if(findTransformIndex(i) == -1 && new_data_maps->at(i) == true)
      {
        //here is something terrible wrong
        //todo fix it.
        ROS_DEBUG("Computing Transform, robotIndex: %lu",i);
        computeTransform(0);
        //here draw if debug
      }
    }
  }
  if(map_data->size() > 0 && transforms->size() > 0)
  {
    ROS_DEBUG("Merging other maps into global map in callback_global_pub,sizeTransforms:%lu,sizeRobots:%lu",transforms->size(),robots->size());
    for(int i = 0; i < transforms->size(); i++)
    {
      if(new_data_maps->at(findRobotIndex(i)) == false)
      {
        int robotIndex = findRobotIndex(i);
        ROS_DEBUG("Skipping %s,no new Data",robots->at(robotIndex).c_str());
        continue;
      }
      nav_msgs::OccupancyGrid * whole_map = map_data->at(i);
      std::string name = whole_map->header.frame_id;
      ROS_DEBUG("Merging [%s] map",name.c_str());
      nav_msgs::OccupancyGrid * map_to_merge = new nav_msgs::OccupancyGrid();
      map_to_merge->data = whole_map->data;
      map_to_merge->info = whole_map->info;
      map_to_merge->header = whole_map->header;
      ROS_DEBUG("In mergeMaps MapData:%i,Transform:%i|size MapData:%lu,size Tranform:%lu",
          i,
          findTransformIndex(i),
          map_data->size(),
          transforms->size());
      updateMap (map_to_merge,findTransformIndex(i));
      mergeMaps(map_to_merge);

      delete map_to_merge;
    }
  }

  ROS_DEBUG("Publishing global_map, contains data of local map and %lu other maps| i am having %lu other maps",
      transforms->size(),
      map_data->size());
  if(global_map != NULL && global_map_ready == true)
  {
    bool newData = false;
    for(int i = 0; i < new_data_maps->size(); i++)
    {
      newData = new_data_maps->at(i);
      if(newData)
        break;
    }
    if(local_map_new_data || newData)
    {
      updateMapArea(-1,local_map);
    }
    // updateMapArea(-1,local_map,true);
    pub.publish(*global_map);
    ROS_DEBUG("!!!After publishing global map!!!");
  }
  else
  {
    ROS_ERROR("!!!Global map not ready!!!");
  }
  return;
}

void MapMerger::callback_send_position(const ros::TimerEvent &e)
{
  ROS_DEBUG("Sending Position, x:%f|y:%f",cur_position->pose.position.x,cur_position->pose.position.y);
  processPosition(cur_position);

  ROS_DEBUG("Publish my position");
  visualization_msgs::Marker m;
  m.header.frame_id = local_map_frame_id;
  m.header.stamp = ros::Time::now();
  m.pose.position.x = cur_position->pose.position.x;
  m.pose.position.y = cur_position->pose.position.y;
  m.pose.position.z = 0;

  m.scale.x = 0.2;
  m.scale.y = 0.2;
  m.scale.z = 0.2;


  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.header.seq = pos_seq_my++;
  m.id = pos_seq_my++;
  pos_seq_my++;
  m.color.a = 1.0;
  m.color.r = 1.0;

  pos_array_my.markers.push_back(m);
  my_pos_pub.publish(pos_array_my);
}

void MapMerger::callback_recompute_transform(const ros::TimerEvent &e)
{
  if(transforms->size() == 0)
  {
    ROS_DEBUG("No transforms to recompute");
    return;
  }
  ROS_INFO("Recompute transforms");

  int newTransforms = 0;
  for(int i = 0; i < map_data->size(); i++)
  {
    if(new_data_maps->at(i) || force_recompute_all == true)
    {
      ROS_DEBUG("New data at index:%i, recomputing Transform",i);
      bool success = recomputeTransform(i);
      if(success)
        newTransforms++;
    }
  }

  if(newTransforms == 0)
    return;
  else
  {
    global_map_ready = false;
    global_map->data.resize(local_map->data.size());
    std::copy(local_map->data.begin(),local_map->data.end(),global_map->data.begin());
    for(int i = 0; i < map_data->size(); i++)
    {
      nav_msgs::OccupancyGrid *whole_map = map_data->at(i);
      nav_msgs::OccupancyGrid *map_to_merge = new nav_msgs::OccupancyGrid();
      map_to_merge->data = whole_map->data;
      map_to_merge->info = whole_map->info;
      map_to_merge->header = whole_map->header;

      updateMap(map_to_merge, findTransformIndex(i));
      mergeMaps(map_to_merge);
      delete map_to_merge;
      new_data_maps->at(i) = false;
    }
    global_map_ready = true;
  }
}

void MapMerger::callback_send_map(const ros::TimerEvent &e)
{
  if(local_map_new_data == false)
  {
    ROS_DEBUG("Do not send map,no new local map data");
    return;
  }

  if(robots->size() == 0)
  {
    ROS_WARN("Do not send map, no robots");
    return;
  }

  if(local_map->data.size() == 0 || local_map_old->data.size() == 0)
  {
    ROS_WARN("Do not send map, local map contains no data");
    return;
  }
  local_map_new_data = false;
  if(debug)
  {
    cv::imwrite("local.pgm",mapToMat(local_map));
    cv::imwrite("old.pgm",mapToMat(local_map_old));
  }
#ifndef DEA_OPT_MAP_CHANGED
  int min_x = 9999;
  int min_y = 9999;
  int max_x = -1;
  int max_y = -1;
  //get map parts to send;
  int index;
  ROS_DEBUG("Check if map changed");
  //+=2 because nearly no differnce to +=1 but lowers number
  //of iterations per 75%
  for(int row = 0; row < local_map->info.height;row+=2)
  {
    for(int collum = 0; collum < local_map->info.width;collum+=2)
    {
      index = row*local_map->info.width + collum;
      if(local_map->data[index]!= local_map_old->data[index])
      {
        if(min_x > row)
          min_x = row;
        if(min_y > collum)
          min_y = collum;
        if(max_x < row)
          max_x = row;
        if(max_y < collum)
          max_y = collum;
      }
    }
  }
  if(min_x == 9999 || min_y == 9999 || max_x == -1 || max_y == -1)
  {
    ROS_DEBUG("map did not changed,skip sending");
    return;
  }
#else
  min_x = 0;
  min_y = 0;
  max_x = map_width;
  max_y = map_height;
#endif
  if(max_x < min_x || max_y < min_y)
    return;
  ROS_DEBUG("Found values\n min_x:%i|max_x:%i|min_y:%i|max_y:%i",min_x,max_x,min_y,max_y);
  ROS_DEBUG("Merge Local Map");
  mergeMaps(local_map);
  //because that is optimized for a map_part, not for other start values
  //mergeMaps(local_map,min_x -5 ,min_y -5 ,max_x +5, max_x +5 );
  std::vector<int> * containedUpdates = new std::vector<int>();
  containedUpdates->push_back(update_seq);
  //todo:mc_ROBOT
  /* for(int i = 0; i < robots->size(); i++)
     sendMapOverNetwork(robots->at(i),containedUpdates,min_x,min_y,max_x,max_y);
     */
  sendMapOverNetwork("mc_" + robot_name,containedUpdates,min_x,min_y,max_x,max_y);


  delete containedUpdates;
  ROS_DEBUG("Sended local map over network,adding updateentry for update number:%i\n\t\t\tminx:%i\tmaxx:%i\tminy:%i\tmaxy:%i",local_map->header.seq,min_x,max_x,min_y,max_y);
  update_list->push_back(new UpdateEntry(update_seq,min_x,min_y,max_x,max_y));
  update_seq++;
  if(local_map->data.size() != local_map_old->data.size())
  {
    ROS_FATAL("Local map changed size, not implemented yet");
    //need to delete all and to remerge the maps
    ROS_INFO("Deleting global_map");
    delete global_map;
    global_map = new nav_msgs::OccupancyGrid();
    global_map->data.resize(local_map->data.size());
    ROS_INFO("Copying local map data into global map");
    std::copy(local_map->data.begin(),local_map->data.end(),global_map->data.begin());
    ROS_INFO("Clearing transforms");
    transforms->clear();
    robots_in_transform->clear();
    nav_msgs::OccupancyGrid * whole_map;
    ROS_INFO("Start recalcing the transformations");
    for(int i = 0; i < robots->size(); i++)
    {
      ROS_INFO("Merge %i in map_data",i);
      computeTransform(i);
      whole_map = map_data->at(i);
      nav_msgs::OccupancyGrid * map_to_merge = new nav_msgs::OccupancyGrid();
      map_to_merge->data = whole_map->data;
      map_to_merge->info = whole_map->info;
      map_to_merge->header = whole_map->header;
      updateMap(map_to_merge,findTransformIndex(i));
      mergeMaps(map_to_merge);
      delete map_to_merge;
      ROS_INFO("Merged %i in map_data",i);
    }
    //exit(-2);
    ROS_INFO("Managed map size change");
  }
  ROS_DEBUG("copy local_map->data to local_map_old->data");
  std::copy(local_map->data.begin(),local_map->data.end(),local_map_old->data.begin());
}

void MapMerger::callback_got_position_network(
    const adhoc_communication::MmRobotPosition::ConstPtr &msg)
{
  geometry_msgs::PoseStamped tmp = msg.get()->position;
  std::string source_host = msg.get()->src_robot;

  ROS_INFO("got a position from: %s", source_host.c_str());
  other_pos_pub = nodeHandle->advertise<geometry_msgs::PoseStamped>
    (robot_prefix + "/other_positions/" + robot_name + "/position", 3);
  other_pos_pub.publish(tmp);
}

void MapMerger::callback_got_position(const nav_msgs::Odometry::ConstPtr &msg)
{
  geometry_msgs::Pose  tmp = msg.get()->pose.pose;
  cur_position->pose = tmp;
}

/**
 * got a map from another robot
 */
void MapMerger::callback_map_other(const nav_msgs::OccupancyGridPtr &msg)
{
  nav_msgs::OccupancyGrid tmp = *msg.get();
  nav_msgs::OccupancyGrid * map = &tmp;

  string robot_name = map->header.frame_id.substr(6, 1);
  ROS_INFO("got a map from: %s", robot_name.c_str());
  ROS_INFO("size: %lu", map->data.size());
  other_map_pub = nodeHandle->advertise<nav_msgs::OccupancyGrid>(robot_prefix + "/other_maps/" + robot_name + "/map", 3);
  other_map_pub.publish(*map);
  ROS_INFO("map published");
}

void MapMerger::processLocalMap(nav_msgs::OccupancyGrid * toInsert,int index)
{
  ROS_DEBUG("Process Local Map");
  if(!has_local_map)
    return;
  local_map_new_data = true;
  if(global_map == NULL)
  {
    local_map->data = toInsert->data;
    local_map->info = toInsert->info;
    local_map->header = toInsert->header;
    local_map->header.frame_id = local_map_frame_id;
    local_map_old->data.resize(toInsert->data.size());
    local_map_old->header = local_map->header;
    std::copy(toInsert->data.begin(),toInsert->data.end(),local_map_old->data.begin());
    local_map_old->info = local_map->info;
    ROS_INFO("%p|%p",&local_map->data,&local_map_old->data);
    global_map = new nav_msgs::OccupancyGrid();
    global_map->data = local_map->data;
    global_map->header = local_map->header;
    global_map->info = local_map->info;
    global_map->info.origin.position.x = g_start_x;
    global_map->info.origin.position.y = g_start_y;

  }
  else
  {
    local_map->data = toInsert->data;
    local_map->info = toInsert->info;
    local_map->header = toInsert->header;

    if(seconds_publish_timer < 2)
    {
      ROS_DEBUG("Updating is Expensive");
      updateMapArea(-1,toInsert);
    }
  }
  return;
}

void MapMerger::processPosition(geometry_msgs::PoseStamped * pose)
{
  //ros::nodeHandle nodeHandle ("~");
  //broadcast to all with map and set frame ID to mine
  ros::ServiceClient client = nodeHandle->serviceClient<adhoc_communication::SendMmRobotPosition>
    (robot_prefix + "/adhoc_communication/send_position");
  adhoc_communication::SendMmRobotPosition exchange;
  exchange.request.topic = position_other_robots_topic;
  exchange.request.dst_robot = "mc_"+robot_name;
  exchange.request.position.position.pose = pose->pose;
  exchange.request.position.src_robot = robot_name;
  //exchange. = robot_name;
  if(client.call(exchange))
  {
    if(exchange.response.status)
      ROS_DEBUG("Could  send position");
    else
      ROS_WARN("Problem sending position");
  }
  else
    ROS_DEBUG("Could not call service to send position");
}

void MapMerger::start()
{
  ros::Subscriber  sub = nodeHandle->subscribe(
      local_map_topic,
      1000,
      &MapMerger::callback_map,
      this);

  ros::Subscriber  sub_control = nodeHandle->subscribe(
      robot_prefix+"/"+control_topic,
      1000,
      &MapMerger::callback_control,
      this);

  ros::Subscriber  sub_other_maps = nodeHandle->subscribe(
      robot_prefix+"/"+topic_over_network,
      1000,
      &MapMerger::callback_map_other,
      this);

  ros::Subscriber  sub_new_robot = nodeHandle->subscribe(
      robot_prefix+"/adhoc_communication/new_robot",
      1000,
      &MapMerger::callback_new_robot,
      this);

  ros::Subscriber  sub_position_local,sub_position_network;

  sub_position_local = nodeHandle->subscribe(robot_prefix+ "/" + position_local_robot_topic,1000,&MapMerger::callback_got_position,this);

  sub_position_network = nodeHandle->subscribe(robot_prefix + "/" + position_other_robots_topic,5,&MapMerger::callback_got_position_network,this);

  if(has_local_map)
    send_position = nodeHandle->createTimer(ros::Duration(seconds_send_position),&MapMerger::callback_send_position,this);
  list_of_positions_publisher = nodeHandle->advertise<adhoc_communication::MmListOfPoints>(robot_prefix+"/"+"all_positions",3);

  pub = nodeHandle->advertise<nav_msgs::OccupancyGrid>("global_map",3);

  my_pos_pub = nodeHandle->advertise<visualization_msgs::MarkerArray>("position_"+robot_name,3);

  if(seconds_recompute_transform != -1)
  {
    ROS_INFO("Create timer to recompute the transformations all %i seconds",seconds_recompute_transform);
    recompute_transform_timer = nodeHandle->createTimer(ros::Duration(seconds_recompute_transform),&MapMerger::callback_recompute_transform,this);
  }

  global_timer_pub = nodeHandle->createTimer(ros::Duration(seconds_publish_timer),&MapMerger::callback_global_pub,this);

  send_map = nodeHandle->createTimer(ros::Duration(seconds_send_timer),&MapMerger::callback_send_map,this);

  ros::ServiceServer transform_srv = nodeHandle->advertiseService("transformPoint",
      &MapMerger::transformPointSRV,
      this);

  ros::ServiceServer log_output_srv = nodeHandle->advertiseService("logOutput",
      &MapMerger::log_output_srv,
      this);

  while(local_map->data.size() == 0 && has_local_map)
  {
    ros::Duration(1).sleep();
    ros::spinOnce();
    ROS_INFO("Local_Map size = 0");
  }

  ask_other_timer = nodeHandle->createTimer(ros::Duration(10),&MapMerger::callback_ask_other_robots,this);

  ros::spin();
}

void MapMerger::mergeMaps(nav_msgs::OccupancyGrid *mapToMerge, int min_x, int min_y, int max_x, int max_y)
{
  ROS_DEBUG("Start merging map: [%s]",mapToMerge->header.frame_id.c_str());
  if(global_map == NULL)
  {
    ROS_INFO("Stop merging maps, globl map is null");
    return;
  }
  int collums,rows;
  if(max_x == -1 || max_y == -1)
  {
    rows    =  mapToMerge->info.height;
    collums =  mapToMerge->info.width;
  }
  else
  {
    rows    =  max_x;
    collums =  max_y;
  }
  int start_x = min_x;
  int start_y = min_y;
  //loop to merge the maps, all of the maps are overlapping becouse they got transformed in the
  //calcOffsets Method. if the cur_map_to_merge knows something the local map do not know -> i
  // set the value
  int index_map_to_merge;
  int index_global_map;
  if(mapToMerge->header.frame_id != local_map_frame_id)
    if(min_x < 0 || min_y < 0)
    {
      ROS_WARN("Map starts in negative area");
      return;
    }
  for(int row = start_x; +row < rows ; row++)
  {
    for(int collum = start_y; collum < collums;collum++)
    {
      index_map_to_merge = (row - start_x) * (collums - start_y) + (collum - start_y);
      index_global_map = row * map_width + collum;

      if(mapToMerge->data[index_map_to_merge] != -1)
        if(local_map->data[index_global_map] == -1)
          global_map->data[index_global_map] = mapToMerge->data[index_map_to_merge];
    }
  }
  ROS_DEBUG("Merged maps succesfully");
}

cv::Mat MapMerger::mapToMat(const nav_msgs::OccupancyGrid *map)
{
  Mat im(map->info.height, map->info.width, CV_8UC1);

  if (map->info.height*map->info.width != map->data.size())
  {
    ROS_ERROR("Data size doesn't match width*height: width = %d, height = %d, data size = %zu", map->info.width, map->info.height, map->data.size());
  }

  // transform the map in the same way the map_saver component does
  for (size_t i=0; i < map->info.height*map->info.width; i++)
  {
    if (map->data.at(i) == 0)
    {
      im.data[i] = 255;
    }
    else
    {
      if(map->data.at(i) == 100)
      {
        im.data[i] = 0;
      }
      else
      {
        im.data[i] = 255;
      }
    }
  }

  return im;
}

nav_msgs::OccupancyGrid* MapMerger::matToMap(const Mat mat, nav_msgs::OccupancyGrid *forInfo)
{
  nav_msgs::OccupancyGrid* toReturn = forInfo;
  for(size_t i=2; i<toReturn->info.height * toReturn->info.width;i++)
  {
    //toReturn->data.push_back(mat.data[i]);
    if(mat.data[i]== 254)//KNOWN
      toReturn->data[i]=0;
    // here it is <10 and not 0 (like in mapToMat), becouse otherwise we loose much
    //walls etc. but so we get a little of wrong information in the unkown area, what is
    // not so terrible.
    // else if(mat.data[i] > -1 && mat.data[i] < 50) toReturn->data[i]=100; //WALL
    else toReturn->data[i] = -1; //UNKOWN
  }
  return toReturn;
}

cv::Mat MapMerger::transformImage(Mat img1,Mat trans)
{
  // create storage for new image and get transformations
  Mat image(img1.size(), img1.type());
  cv::Size s;
  s.height = image.rows + 0;
  s.width = image.cols + 0;

  //should be faster than the other
  warpAffine(img1,image,trans,s,INTER_NEAREST,BORDER_TRANSPARENT);
  //warpAffine(img1,image,trans,s,INTER_AREA,BORDER_DEFAULT);
  //cv::imwrite("test.pgm",image);
  return image;
}

/**
 * does the transform specified in the map to update, then writes into mapToUpdate
 */
void MapMerger::updateMap(nav_msgs::OccupancyGrid *mapToUpdate,int index_of_transform)
{
  ROS_DEBUG("Start updateMap");
  if(index_of_transform == -1)
  {
    ROS_WARN("No transformation available for:[%s]",mapToUpdate->header.frame_id.c_str());
    return;
  }
  //convert the map to a mat, then transform the image, and then convert the mat to a map
  //and merge them
  ROS_DEBUG("Update:[%s]",mapToUpdate->header.frame_id.c_str());

  cv::Mat img = mapToMat(mapToUpdate);

  img = transformImage(img,transforms->at(index_of_transform));
  /*if(debug)
    cv::imwrite("transed_img.pgm",img);*/
  mapToUpdate =  matToMap(img,mapToUpdate);
}

nav_msgs::OccupancyGrid* MapMerger::getMapPart(int map_index, int start_x, int start_y, int width, int height)
{
  ROS_DEBUG("Start in getMapPart, creating map part, min_x:%i|max_x:%i|min_y:%i|max_y:%i|Index:%i",start_x,start_x+height,start_y,start_y+width,map_index);

  nav_msgs::OccupancyGrid * tmp = local_map;
  while(true)
  {
    nav_msgs::OccupancyGrid * tmp = local_map;
    if(tmp->data.size() > 0)
    {
      ROS_DEBUG("TMP larger than 0");
      break;
    }
    else
    {
      ROS_DEBUG("TMP 0");
      ros::Duration(2).sleep();
    }
  }
  if(map_index != -1)
  {
    tmp = map_data->at(map_index);
  }
  int sizePart = (start_x + height)*(start_y+ width);
  if(sizePart > tmp->data.size())
  {
    ROS_ERROR("MapWidth:%i\tMapHeight:%i",map_width,map_height);
    ROS_ERROR("TmpWidth:%i\tTmpHeight:%i\tstartx:%i\tstarty:%i",tmp->info.width,tmp->info.height,start_x,start_y);
    ROS_ERROR("Part is too large \n\t\t\t\tsizePart:%i\ttmp size:%lu\twidth:%i\theight:%i",
        sizePart,tmp->data.size(),width,height);
    return NULL;
  }
  nav_msgs::OccupancyGrid* part = new nav_msgs::OccupancyGrid();
  for(int row = 0; row < height;row ++)
  {
    for(int collum = 0; collum < width;collum++)
    {
      part->data.push_back(tmp->data[(row+start_x)*tmp->info.width+(collum + start_y)]);
    }
  }
  part->header = tmp->header;
  part->info.origin.position.x = start_x;
  part->info.origin.position.y = start_y;
  part->info.height = height;
  part->info.width = width;
  return part;
}

void MapMerger::updateMapArea(int map_index, nav_msgs::OccupancyGrid *newData, bool clear)
{
  nav_msgs::OccupancyGrid * tmp;
  int start_x = newData->info.origin.position.x;
  int start_y = newData->info.origin.position.y;
  if(map_index == -2)
    tmp = local_map;
  if(map_index == -1)
  {
    tmp = global_map;
    if(has_local_map)
    {
      start_x = 0;
      start_y = 0;
    }
  }
  if(map_index > -1)
    tmp = map_data->at(map_index);

  ROS_DEBUG("Updating map Area, new data from map:%s|x:%i|y:%i",newData->header.frame_id.c_str(),start_x,start_y);
  int width = newData->info.width;
  int height = newData->info.height;
  if(clear)
  {
    width = 10;
    height = 10;
  }
  for(int row = 0; row < height; row++)
  {
    for(int collum = 0; collum < width; collum++)
    {
      int i_data =row * width + collum ;
      if(newData->data[i_data] == -1 && clear == false)
        continue;
      //may segfault, because every map has a other size
      int i_global = (row+start_x)*tmp->info.width+(collum + start_y);
      tmp->data[i_global] = newData->data[i_data];
    }
  }
  tmp->header.frame_id = newData->header.frame_id;
}

bool MapMerger::recomputeTransform(int mapDataIndex)
{
  int sum_elements = std::accumulate(local_map->data.begin(),local_map->data.end(),0);
  if(local_map->data.size()== 0
      || sum_elements == (-1) * local_map->data.size())
  {
    ROS_WARN("Local Map is empty, break up transforming");
    return false;
  }
  nav_msgs::OccupancyGrid * map = map_data->at(mapDataIndex);
  std::string map_name = map->header.frame_id;
  ROS_DEBUG("Restransforming : [%s]",map_name.c_str());
  cv::Mat img_local = mapToMat(local_map);
  cv::Mat img_other = mapToMat(map);

  int transIndex = findTransformIndex(mapDataIndex);
  cv::Mat newTrans;
  bool worked;
  if(transIndex == -1)
  {
    StitchedMap mapStiched(img_local,img_other,max_trans_robot,max_rotation_robot,5,cv::Mat());
    worked = mapStiched.works;
    if(worked)
    {
      newTrans = mapStiched.H;
      lastTrans = mapStiched.cur_trans;
    }
  }
  else
  {
    StitchedMap mapStiched(img_local,img_other,max_trans_robot,max_rotation_robot,5,transforms->at(transIndex));
    worked = mapStiched.works;
    if(worked)
    {
      newTrans = mapStiched.H;
      lastTrans = mapStiched.cur_trans;
    }
  }
  //calculates the transformation between the local map and all the other maps, and
  //then updates the map data.
  if(worked)
  {
    ROS_DEBUG("Storing new Transform for %s in array",map->header.frame_id.c_str());
    if(findTransformIndex(mapDataIndex) == -1)
    {
      ROS_DEBUG("Appending transform for robot:%i",mapDataIndex);
      transforms->push_back(newTrans);
      robots_in_transform->push_back(robots->at(mapDataIndex));
    }
    else
    {
      ROS_DEBUG("Updateing transform for robot:%i",mapDataIndex);
      transforms->at(findTransformIndex(mapDataIndex)) = newTrans;
    }
  }
  return worked;
}

void MapMerger::computeTransform(int mapDataIndex)
{
  ROS_DEBUG("Start computeTransform for mapDataIndex:%i",mapDataIndex);
  int sum_elements = std::accumulate(local_map->data.begin(),local_map->data.end(),0);
  if(local_map->data.size()== 0
      || sum_elements == (-1) * local_map->data.size())
  {
    ROS_WARN("Local Map is empty, break up transforming");
    return;
  }
  nav_msgs::OccupancyGrid * whole_map = map_data->at(mapDataIndex);

  //this check only here to test if günther node send all data
  sum_elements = std::accumulate(whole_map->data.begin(),whole_map->data.end(),0);
  if(whole_map->data.size() == 0
      || sum_elements == (-1) * whole_map->data.size() )
  {
    ROS_WARN("Other map is empty, break up transforming|%i",sum_elements);
    return;
  }
  //ROS_WARN("INFO sum_elements other map is %i",sum_elements);
  //end check


  new_data_maps->at(mapDataIndex) = false;
  std::string map_name = whole_map->header.frame_id;
  ROS_DEBUG("Transforming : [%s]",map_name.c_str());
  cv::Mat img_local = mapToMat(local_map);
  cv::Mat img_other = mapToMat(whole_map);
  StitchedMap mapStiched(img_local,img_other,max_trans_robot,max_rotation_robot,5,cv::Mat());
  //calculates the transformation between the local map and all the other maps, and
  //then updates the map data.
  if(!mapStiched.works)
  {
    return;
  }
  else
  {
    if(findTransformIndex(mapDataIndex) == -1)
    {
      transforms->push_back(mapStiched.H);
      robots_in_transform->push_back(robots->at(mapDataIndex));
      lastTrans = mapStiched.cur_trans;
    }
    else
    {
      ROS_DEBUG("Updateing transform for robot:%i",mapDataIndex);
      transforms->at(findTransformIndex(mapDataIndex)) = mapStiched.H;
      lastTrans = mapStiched.cur_trans;
    }
    //imwrite("img_local_map",mapToMat(local_map));
    //imwrite("img_other",mapToMat(map_data->at(index)));

    nav_msgs::OccupancyGrid * map_to_merge = new nav_msgs::OccupancyGrid();
    map_to_merge->data = whole_map->data;
    map_to_merge->info = whole_map->info;
    map_to_merge->header = whole_map->header;
    updateMap(map_to_merge,findTransformIndex(mapDataIndex));
    mergeMaps(map_to_merge);
    delete map_to_merge;
  }
  return;
}

void MapMerger::makeEmptyMapData(string robot_name,int height,int width,float resolution)
{
  ROS_DEBUG("Creating emptyMapData for [%s]",robot_name.c_str());
  nav_msgs::OccupancyGrid * tmp = new nav_msgs::OccupancyGrid();
  tmp->info.height = height;
  tmp->info.width = width;
  tmp->info.origin.orientation.w = 1;
  tmp->header.frame_id = robot_name;
  tmp->info.resolution =resolution; //should not be hard coded
  ROS_DEBUG("Filling emptyMapData for [%s]",robot_name.c_str());
  for(int i = 0; i < tmp->info.height * tmp->info.width; i++)
  {
    tmp->data.push_back(-1);
  }
  ROS_DEBUG("Pushing  emptyMapData for [%s] onto array",robot_name.c_str());
  map_data->push_back(tmp);
  ROS_DEBUG("Creating bool for checking if new data");
  new_data_maps->push_back(false);
  if(exchange_position)
  {
    pos_seq_other->push_back(0);
    pos_array_other->push_back(visualization_msgs::MarkerArray());
    pos_pub_other->push_back(nodeHandle->advertise<visualization_msgs::MarkerArray>("position_"+robot_name,3));
  }
}

void MapMerger::sendLocationOverNetwork(std::string destination) {
  ros::ServiceClient client = 
    nodeHandle->serviceClient<adhoc_communication::SendMmRobotPosition>
    (robot_prefix + "/adhoc_communication/send_position");
  adhoc_communication::SendMmRobotPosition exchange;
  exchange.request.topic = position_other_robots_topic;
  exchange.request.dst_robot = destination;
  exchange.request.position.position.pose = cur_position->pose;
  exchange.request.position.src_robot = robot_name;

  exchange.request.position.position.header.frame_id = robot_name + "/map";

  if(!client.call(exchange)) {
    ROS_DEBUG("Could not call service to send map");
    return;
  }

  if(!exchange.response.status) {
    ROS_WARN("Destination host unreachable [%s](if nothing -> boradcast)",
        destination.c_str());
    return;
  }
}

void MapMerger::sendMapOverNetwork(
    string destination, 
    std::vector<int>* containedUpdates, 
    int start_row, 
    int start_collum,
    int end_row,
    int end_collum)
{
  //ros::nodeHandle nodeHandle ("~");
  ros::ServiceClient client;
  bool isWholeMap = false;
  client = nodeHandle->serviceClient<adhoc_communication::SendMmMapUpdate>
    (robot_prefix + "/adhoc_communication/send_map_update");
  std::string service = robot_prefix + "/adhoc_communication/send_map_update";
  if(end_row == -1 || end_collum == -1)
  {
    ROS_DEBUG("Send whole map");
    end_row = map_height;
    end_collum = map_width;
    start_row = 0;
    start_collum = 0;
    isWholeMap = true;
  }
  ROS_DEBUG("Send map using:%s \n\t\t\t\t the normal way min_x:%i|max_x:%i|min_y:%i|max_y:%i",service.c_str(),start_row,end_row,start_collum,end_collum);
  /**
todo:
values for the mapPart are used wrong, always creates one great map part if i just send 688 x 688 packets,
fix the problem
*/
  int height,width;
  height = size;
  width = size;
  for(int row = start_row; row < end_row;row+=size)
  {
    for(int collum = start_collum; collum < end_collum;collum+=size)
    {
      if(row + height > end_row)
        height = end_row - row;
      if(collum + width > end_collum)
        width = end_collum - collum;
      ROS_DEBUG("Creating map part t");
      nav_msgs::OccupancyGrid * t = this->getMapPart(-1,row,collum,width,height);
      ROS_DEBUG("Checking data of  map part t,size:%lu",t->data.size());

      adhoc_communication::SendMmMapUpdate exchange;
      exchange.request.topic = topic_over_network;
      exchange.request.map_update.src_robot = robot_hostname;
      exchange.request.map_update.map.info = t->info;
      exchange.request.map_update.map.header = t->header;
      exchange.request.map_update.map.header.seq = update_seq;
      exchange.request.map_update.map.header.frame_id = robot_name.c_str();
      exchange.request.map_update.map.data = t->data;
      exchange.request.map_update.update_numbers = *containedUpdates;
      std::string hostname = destination;
      exchange.request.dst_robot = destination;
      ROS_DEBUG("Send Map to %s(if nothing -> boradcast) ,frame id:%s| topic:[%s]|data<_size:%lu",
          destination.c_str(),t->header.frame_id.c_str(),
          topic_over_network.c_str(), exchange.request.map_update.map.data.size());
      if(client.call(exchange))
      {
        if(exchange.response.status)
          if(destination == "")
            ROS_DEBUG("Sended Map to:all,topic:%s|r:%i;c:%i",exchange.request.topic.c_str(),row,collum);
          else
            ROS_DEBUG("Sended Map to:%s,topic:%s|r:%i;c:%i",exchange.request.dst_robot.c_str(),exchange.request.topic.c_str(),row,collum);
        else
          ROS_WARN("Destination host unreachable [%s](if nothing -> boradcast)",hostname.c_str());
      }
      else
        ROS_DEBUG("Could not call service to send map");
    }
  }
}

int MapMerger::findTransformIndex(int robot_index)
{
  if(transforms->size() == 0)
  {
    return -1;
  }
  for(int r = 0; r < robots_in_transform->size(); r++)
  {
    if(robots_in_transform->at(r)==robots->at(robot_index))
    {
      return r;
    }
  }
  return -1;
}

int MapMerger::findRobotIndex(int transform_index)
{
  for(int i = 0; i < robots_in_transform->size(); i++)
  {
    if(robots_in_transform->at(transform_index) == robots->at(i))
      return i;
  }
  return -1;
}

bool MapMerger::createLogPath()
{
  full_log_path = log_path + std::string("/map_merger/") + robot_name;
  ROS_INFO("Creating log path \"%s\".", full_log_path.c_str());
  boost::filesystem::path boost_log_path(full_log_path.c_str());
  if(!boost::filesystem::exists(boost_log_path))
    try{
      if(!boost::filesystem::create_directories(boost_log_path))
        ROS_ERROR("Cannot create directory \"%s\".", full_log_path.c_str());
    }catch(const boost::filesystem::filesystem_error& e)
  {
    ROS_ERROR("Cannot create path \"%s\".", full_log_path.c_str());
    return false;
  }
  return true;
}

/**
 * \brief Saves progress to log.
 * Saves log information to the hard drive. At this point log information may include
 *  * global map
 *  * local map
 */
bool MapMerger::log_output_srv(map_merger::LogMaps::Request &req, map_merger::LogMaps::Response &res)
{
  ROS_DEBUG("--> Top of service log_output_srv");
  if(!createLogPath())
    return false;

  unsigned int local_progress = 0;
  unsigned int global_progress = 0;
  std::string file;
  ros::Duration ros_time = ros::Time::now() - time_start;

  if(req.log & LOG_GLOBAL_MAP)
  {
    if(!global_map)
      ROS_WARN("No global map. Skipping save global map.");
    else
    {
      // log the global map 
      ROS_DEBUG("Storing global file...");
      file = full_log_path + std::string("/global.pgm");
      cv::Mat glo = mapToMat(global_map);
      circle(glo,Point(glo.rows/2,glo.cols/2),5,Scalar(0,0,0),2);
      if(global_map_ready)
        cv::imwrite(file.c_str(),glo);
      ROS_DEBUG("Successfully logged global map.");
    }
  }
  if(req.log & LOG_LOCAL_MAP)
  {
    if(!local_map)
      ROS_WARN("No global map. Skipping save local map.");
    else
    {
      // log the local map
      ROS_DEBUG("Storing local file...");
      file = full_log_path + std::string("/local.pgm");

      cv::Mat loc = mapToMat(local_map);
      circle(loc,Point(loc.rows/2,loc.cols/2),5,Scalar(0,0,0),2);
      cv::imwrite(file.c_str(),loc);

      ROS_DEBUG("Successfully logged local map.");
    }
  }
  if(req.log & LOG_LOCAL_MAP_PROGRESS)
  {
    if(!local_map)
      ROS_WARN("No local map. Skipping save local map progress.");
    else
    {
      // log the local map progress, i.e., the number of free spaces in the local map
      ROS_DEBUG("Logging local map progress");
      for(int i=0; i<local_map->data.size(); i++)
      {
        if(local_map->data[i] == 0)
          local_progress++;
      }
    }
  }
  if(req.log & LOG_GLOBAL_MAP_PROGRESS)
  {
    if(!global_map)
      ROS_WARN("No global map. Skipping save global map progress.");
    else
    {
      // log the global map progress, i.e., the number of free spaces in the local map
      ROS_DEBUG("Logging global map progress");
      for(int i=0; i<global_map->data.size(); i++)
      {
        if(global_map->data[i] == 0)
          global_progress++;
      }
    }
  }


  if(req.log & LOG_GLOBAL_MAP_PROGRESS || req.log & LOG_LOCAL_MAP_PROGRESS)
  {
    // save progress to file
    file = full_log_path + std::string("/periodical.log");
    std::string file_updates = full_log_path + std::string("/updates_log");
    std::ofstream fs1;
    fs1.open(file_updates.c_str(),std::ios_base::app);
    if(!fs1)
    {
      ROS_ERROR("Cannot create file. Error: %s", strerror(errno));
    }

    for(int i = 0; i < robots->size(); i++)
    {
      std::string rName = robots->at(i);
      fs1 << rName << ": ";
      for(int j = 0 ; j < updateMan->getUpdateListOfrobot(0)->size(); j++)
      {
        int v = updateMan->getUpdateListOfrobot(0)->at(j);
        fs1 << v << "-";
      }
      fs1 << std::endl;
    }
    fs1.close();
    ROS_DEBUG("Logging to file \"%s\".", file.c_str());
    std::ofstream fs;
    fs.open(file.c_str(), std::ios_base::app);
    if(!fs)
    {
      ROS_ERROR("Cannot create file. Error: %s", strerror(errno));
    }
    //todo: write creation_time if needed
    //fs << creation_time << "," << global_progress << "," << local_progress << std::endl;
    fs << ros::Time::now() << "," << global_progress << "," << local_progress << "," << cur_position->pose.position.x << "," << cur_position->pose.position.y << "," << update_list->size() << "," << transforms->size() << std::endl;

    fs.close();

    ROS_DEBUG("Storing local file...");
    file = full_log_path + std::string("/local.pgm");

    cv::Mat loc = mapToMat(local_map);
    circle(loc,Point(loc.rows/2,loc.cols/2),5,Scalar(0,0,0),2);
    cv::imwrite(file.c_str(),loc);

    ROS_DEBUG("Successfully logged local map.");

    ROS_DEBUG("Storing global file...");
    file = full_log_path + std::string("/global.pgm");
    cv::Mat glo = mapToMat(global_map);
    circle(glo,Point(glo.rows/2,glo.cols/2),5,Scalar(0,0,0),2);
    if(global_map_ready)
      cv::imwrite(file.c_str(),glo);
    ROS_DEBUG("Successfully logged global map.");
  }
  ROS_DEBUG("<-- end log_output_srv");
  return true;
}

bool MapMerger::transformPointSRV(map_merger::TransformPoint::Request &req, map_merger::TransformPoint::Response &res)
{
  //need transform
  int index_transform = -1;
  if(req.point.src_robot == robot_name)
    ROS_ERROR("Transform my own point!");
  for(int i = 0; i < robots->size();i++)
  {
    if(robots->at(i) == req.point.src_robot)
    {
      ROS_DEBUG("Found %s with %s at %i",
          robots->at(i).c_str(),
          req.point.src_robot.c_str(),
          i);
      index_transform = findTransformIndex(i);
      break;
    }
  }
  if(index_transform == -1)
  {
    ROS_WARN("Could not transform point, no transform matrix found for %s",
        req.point.src_robot.c_str());
    return false;
  }
  cv::Mat trans = transforms->at(index_transform);
  cv::Point org_point(map_width / 2 +req.point.x / 0.05,
      map_height / 2 +req.point.y / 0.05);
  cv::Point homogeneous;
  std::vector<cv::Point> inPts,outPts;
  inPts.push_back(org_point);
  outPts.push_back(homogeneous);
  cv::Size s;
  s.height = map_height; //* 0.05;
  s.width = map_width ;//* 0.05;
  cv::transform(inPts,outPts,trans);
  res.point.x = (outPts.at(0).x - map_width / 2) * 0.05;
  res.point.y = (outPts.at(0).y - map_height / 2) * 0.05;
  res.point.src_robot = robot_hostname;
  return true;
}

void MapMerger::sendControlMessage(std::vector<int>* updateNumbers,std::string dest)
{
  ros::ServiceClient client = nodeHandle->serviceClient<adhoc_communication::SendMmControl>(robot_prefix +"/adhoc_communication/send_control_message");
  adhoc_communication::SendMmControl exchange;
  exchange.request.dst_robot= dest;
  exchange.request.topic = control_topic;
  exchange.request.msg.src_robot = robot_hostname;
  exchange.request.msg.update_numbers = *updateNumbers;
  ROS_INFO("Sending control message to %s on topic %s", dest.c_str(), control_topic.c_str());
  if(!client.call(exchange))
  {
    ROS_DEBUG("Could not call service to send control message");
    return;
  }
  if(!exchange.response.status)
  {
    ROS_WARN("Problem sending control message to %s",dest.c_str());
    return;
  }
}

void MapMerger::callback_ask_other_robots(const ros::TimerEvent &e)
{
  ros::ServiceClient getNeighborsClient = nodeHandle->serviceClient<adhoc_communication::GetNeighbors>
    (robot_prefix+"/adhoc_communication/get_neighbors");
  adhoc_communication::GetNeighbors getNeighbors;

  if(!getNeighborsClient.call(getNeighbors))
  {
    ROS_WARN("Can't call service to get neighbors");
    return;
  }

  if(getNeighbors.response.neigbors.size() <= 0)
  {
    ROS_INFO("No robots are in the network");
    return;
  }

  std::vector<int>* empty = new std::vector<int>();
  for(int i = 0; i < getNeighbors.response.neigbors.size();i++)
  {
    std::string newRobotName = getNeighbors.response.neigbors.at(i);
    if(newRobotName == robot_name)
    {
      ROS_WARN("Got my own name as a neigbor!");
      continue;
    }
    sendControlMessage(empty,newRobotName);
  }
  delete empty;
}
