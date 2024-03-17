#include <string>
#include <vector>
#include <apsrc_lead_vehicle_detection/apsrc_lead_vehicle_detection_nodelet.hpp>



namespace apsrc_lead_vehicle_detection
{
ApsrcLeadVehicleDetectionNl::ApsrcLeadVehicleDetectionNl()
{
}

ApsrcLeadVehicleDetectionNl::~ApsrcLeadVehicleDetectionNl()
{
}

void ApsrcLeadVehicleDetectionNl::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  loadParams();

  // Publishers
  lead_vehicle_pub_ = nh_.advertise<apsrc_msgs::LaedVehicle>("/lead_vehicle/track", 10, true);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/lead_vehicle/markers", 10, true);

  // Subscribers
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &ApsrcLeadVehicleDetectionNl::velocityCallback, this);
  radar_point_cloud_sub_ = nh_.subscribe(radar_pc_topic_name_, 1, &ApsrcLeadVehicleDetectionNl::radarPointCloudCallback, this);
  lidar_detection_sub_ = nh_.subscribe(lidar_objs_topic_name_, 1, &ApsrcLeadVehicleDetectionNl::lidarObjectDetectionCallback, this);
}

void ApsrcLeadVehicleDetectionNl::loadParams()
{
  pnh_.param<std::string>("radar_pc_topic", radar_pc_topic_name_, "/radar_fc/as_tx/objects");
  pnh_.param<std::string>("lidar_objs_topic", lidar_objs_topic_name_, "/detection/lidar_objects");
  pnh_.param("roi_max_lat", roi_max_lat_, 2.0);
  pnh_.param("roi_min_lng", roi_min_lng_, 1.0);
  pnh_.param("roi_max_dist", roi_max_dist_, 2.0);
  pnh_.param("min_time_gap", min_time_gap_, 2.0);
  pnh_.param("min_lead_ego_speed_rate", min_lead_ego_speed_rate_, 2.0);
  pnh_.param("visulization", visualization_, true);
  
  ROS_INFO("Parameters Loaded");
}

void ApsrcLeadVehicleDetectionNl::radarPointCloudCallback(const derived_object_msgs::ObjectWithCovarianceArray::ConstPtr& radar_pc)
{
  std::vector<apsrc_lead_vehicle_detection::radarPoint> point_array = {};
  derived_object_msgs::ObjectWithCovariance ref;
  ref.pose.pose.position.x = 0;
  ref.pose.pose.position.y = 0;
  bool found_any = false;

  size_t closes_object_loc_id = 0;
  bool found = false;
  if (radar_lead_found_){
    if(lidar_lead_found_){
      std::vector<size_t> within_range(10);
      found_any = false;
      for (size_t i = 0; i < radar_pc->objects.size(); ++i){
        float dist = apsrc_lead_vehicle_detection::lidar_radar_dist_func(lidar_lead_t_, radar_pc->objects[i]);
        if (dist < roi_max_dist_){
          within_range.push_back(i);
          found_any = true;
        }
      }
      if (found_any){
        double min_dist = apsrc_lead_vehicle_detection::radar_objs_dist_func(lead_, radar_pc->objects[within_range[0]]);
        for(size_t j = 0; j < within_range.size(); ++j){
          float dist = apsrc_lead_vehicle_detection::radar_objs_dist_func(lead_, radar_pc->objects[within_range[j]]);
          if (dist <= min_dist){
            min_dist = dist;
            closes_object_loc_id = within_range[j];
            found = true;
          }
        } 
      }
    }else{
      std::vector<size_t> within_range(10);
      found_any = false;
      for (size_t i = 0; i < radar_pc->objects.size(); ++i){
        float dist = apsrc_lead_vehicle_detection::radar_objs_dist_func(lead_, radar_pc->objects[i]);
        if (dist < roi_max_dist_){
          within_range.push_back(i);
          found_any = true;
        }
      }
      if (found_any){
        double min_dist = apsrc_lead_vehicle_detection::radar_objs_dist_func(ref, radar_pc->objects[within_range[0]]);
        for(size_t j = 0; j < within_range.size(); ++j){
          float dist = apsrc_lead_vehicle_detection::radar_objs_dist_func(ref, radar_pc->objects[within_range[j]]);
          if (dist <= min_dist){
            min_dist = dist;
            closes_object_loc_id = within_range[j];
            found = true;
          }
        }
      }
    }
  } else {
    if(lidar_lead_found_){
      std::vector<size_t> within_range(10);
      found_any = false;
      for (size_t i = 0; i < radar_pc->objects.size(); ++i){
        float dist = apsrc_lead_vehicle_detection::lidar_radar_dist_func(lidar_lead_t_, radar_pc->objects[i]);
        if (dist < roi_max_dist_){
          within_range.push_back(i);
          found_any = true;
        }
      }
      if (found_any){
        double min_dist = apsrc_lead_vehicle_detection::radar_objs_dist_func(ref, radar_pc->objects[within_range[0]]);
        for(size_t j = 0; j < within_range.size(); ++j){
          float dist = apsrc_lead_vehicle_detection::radar_objs_dist_func(ref, radar_pc->objects[within_range[j]]);
          if (dist <= min_dist){
            min_dist = dist;
            closes_object_loc_id = within_range[j];
            found = true;
          }
        }
      }
    }else{
      found = false;
    }
  }
  
  apsrc_msgs::LaedVehicle msg;
  msg.emergency_stop = stop_now_;
  if (found && radar_pc->objects[closes_object_loc_id].pose.pose.position.x >= roi_min_lng_ &&
      std::abs(radar_pc->objects[closes_object_loc_id].pose.pose.position.y) <= 2*roi_max_lat_){
    radar_lead_found_ = true;
    msg.lead_detected = true;
    lead_ = radar_pc->objects[closes_object_loc_id];
  } else {
    radar_lead_found_ = false;
    msg.lead_detected = false;
    confidence_ = 0;
  }

  msg.header.frame_id = "radar_fc";
  msg.header.stamp = ros::Time::now();
  if (msg.lead_detected){
    msg.range = apsrc_lead_vehicle_detection::radar_objs_dist_func(ref, lead_);
    msg.speed = lead_.twist.twist.linear.x + current_velocity_;
    if (msg.speed < current_velocity_ * min_lead_ego_speed_rate_){
      msg.emergency_stop = true;
    }
    msg.speed_mph = msg.speed * 2.23694;
    msg.confidence = confidence_;
    msg.lidar = lidar_lead_found_;
  }
  lead_vehicle_pub_.publish(msg);

  if (visualization_){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "radar_fc";
    marker.header.stamp = msg.header.stamp;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = .5;
    marker.color.r = 100;
    marker.color.b = 0;
    marker.color.g = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;

    visualization_msgs::Marker text; 
    text.header.frame_id = "radar_fc";
    text.header.stamp = ros::Time::now();
    text.id = 1;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position.z = 3; 
    text.pose.orientation.x = 0.0;
    text.pose.orientation.y = 0.0;
    text.pose.orientation.z = 0.0;
    text.pose.orientation.w = 1.0;
    text.scale.x = 1.0;
    text.scale.y = 1.0;
    text.scale.z = 1.0;
    text.color.r = 0.0;
    text.color.g = 1.0;
    text.color.b = 0.0;
    text.color.a = 1.0;

    if (msg.lead_detected){
      marker.pose.position.x = lead_.pose.pose.position.x;
      marker.pose.position.y = lead_.pose.pose.position.y;
      marker.action = visualization_msgs::Marker::ADD;     
      text.pose.position.x = lead_.pose.pose.position.x;
      text.pose.position.y = lead_.pose.pose.position.y;
      std::string stop_str = msg.emergency_stop ? "true" : "false";
      text.text = "speed: " + std::to_string(msg.speed) + "m/s\n" +
      std::to_string(msg.speed_mph) + "mph\n" + 
      "dist: "  + std::to_string(msg.range) + "m\n" + 
      "Conf: " + std::to_string(msg.confidence) + "%\n" +
      "stop_now: " + stop_str;
    } else {
      marker.action = visualization_msgs::Marker::DELETE;
      text.action = visualization_msgs::Marker::DELETE;
    }

    visualization_msgs::Marker rect;
    rect.header.frame_id = "radar_fc";
    rect.header.stamp = ros::Time::now();
    rect.id = 2;
    rect.type = visualization_msgs::Marker::CUBE;
    rect.action = visualization_msgs::Marker::ADD;
    rect.scale.x = current_velocity_ * min_lead_ego_speed_rate_;
    rect.scale.y = 4.0;
    rect.scale.z = 0.1;
    rect.pose.position.x = current_velocity_ * min_lead_ego_speed_rate_ / 2;
    rect.pose.position.y = 0.0;
    rect.pose.position.z = 0.0;
    rect.pose.orientation.x = 0.0;
    rect.pose.orientation.y = 0.0;
    rect.pose.orientation.z = 0.0;
    rect.pose.orientation.w = 1.0;
    rect.color.r = 1.0;
    rect.color.g = 0.0;
    rect.color.b = 0.0;
    rect.color.a = 0.3;
    marker_pub_.publish(marker);
    marker_pub_.publish(text);
    marker_pub_.publish(rect);
  }
}

void ApsrcLeadVehicleDetectionNl::lidarObjectDetectionCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& lidar_objs)
{
  bool found = false;
  float min_dist;

  if (lidar_lead_found_){
    min_dist = apsrc_lead_vehicle_detection::lidar_objs_dist_func(lidar_objs->objects[0], lidar_lead_);
    size_t closes_object_loc_id = 0;
    for (size_t i = 0; i < lidar_objs->objects.size(); ++i){
      if (ApsrcLeadVehicleDetectionNl::emergency_stop_func(lidar_objs->objects[i])){
        stop_now_ = true;
        return;
      }
      if (lidar_objs->objects[i].label == "car"){
        float dist = apsrc_lead_vehicle_detection::lidar_objs_dist_func(lidar_objs->objects[i], lidar_lead_);
        if (dist <= min_dist){
          min_dist = dist;
          closes_object_loc_id = i;
          found = true;
        }
      }
    }
    if (found && min_dist <= roi_max_dist_){
      lidar_lead_ = lidar_objs->objects[closes_object_loc_id];
      lidar_lead_t_ = lidar_lead_;
      lidar_lead_t_.pose.position.x -= lidar_radar_dist_;
      confidence_ = ++confidence_ > 100 ? 100 : confidence_;
    } else {
      found = false;
    }
  } else {
    confidence_ = --confidence_ < 0 ? 0 : confidence_;
    autoware_msgs::DetectedObject ref = {};
    ref.pose.position.x = 0;
    ref.pose.position.y = 0;
    min_dist = apsrc_lead_vehicle_detection::lidar_objs_dist_func(lidar_objs->objects[0], ref);
    size_t closes_object_loc_id = 0;
    for (size_t i = 0; i < lidar_objs->objects.size(); ++i){
      float dist = apsrc_lead_vehicle_detection::lidar_objs_dist_func(lidar_objs->objects[i], ref);
      if (std::abs(lidar_objs->objects[i].pose.position.y) <= roi_max_lat_ && lidar_objs->objects[i].pose.position.x >= roi_min_lng_ + 3.588){
        if (ApsrcLeadVehicleDetectionNl::emergency_stop_func(lidar_objs->objects[i])){
          stop_now_ = true;
          return;
        }
        min_dist = dist;
        closes_object_loc_id = i;
        found = true;
      }
    }
    if (found) {
      lidar_lead_ = lidar_objs->objects[closes_object_loc_id];
      lidar_lead_t_ = lidar_lead_;
      lidar_lead_t_.pose.position.x -= lidar_radar_dist_;
    }
  }
  lidar_lead_found_ = found;
  stop_now_ = false;
}

void ApsrcLeadVehicleDetectionNl::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity)
{
  current_velocity_ = static_cast<uint16_t>(std::round(std::abs(current_velocity->twist.linear.x)));
  current_velocity_rcvd_time_ = ros::Time::now();
}

bool ApsrcLeadVehicleDetectionNl::emergency_stop_func(autoware_msgs::DetectedObject obj)
{
  if (obj.pose.position.x > lidar_radar_dist_ && obj.pose.position.x < current_velocity_ * min_time_gap_ + lidar_radar_dist_ 
      && std::abs(obj.pose.position.y) <= 2){
    return true;
  }
  return false;
}

}  // namespace apsrc_lead_vehicle_detection

PLUGINLIB_EXPORT_CLASS(apsrc_lead_vehicle_detection::ApsrcLeadVehicleDetectionNl,
                       nodelet::Nodelet);
