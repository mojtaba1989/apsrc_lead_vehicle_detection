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
  lead_vehicle_pub_ = nh_.advertise<apsrc_msgs::LeadVehicle>("/lead_vehicle/track", 10, true);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/lead_vehicle/markers", 10, true);

  // Subscribers
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &ApsrcLeadVehicleDetectionNl::velocityCallback, this);
  radar_point_cloud_sub_ = nh_.subscribe(radar_pc_topic_name_, 1, &ApsrcLeadVehicleDetectionNl::radarPointCloudCallback, this);
  lidar_detection_sub_ = nh_.subscribe(lidar_objs_topic_name_, 1, &ApsrcLeadVehicleDetectionNl::lidarObjectDetectionCallback, this);

  // iterate
  timer_ = nh_.createTimer(ros::Duration(0.05), std::bind(&ApsrcLeadVehicleDetectionNl::timerCallBack, this));
  lidar_x_gain_(0,0) = 1;
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
  std::unique_lock<std::mutex> status_lock(read_lock_);
  if (radar_pc->objects.size() == 0){
    return;
  }
	if (lead_.initilized){
  	size_t closes_object_loc_id = 0;
		float min_dist = apsrc_lead_vehicle_detection::radar_KFobj_dist_func(radar_pc->objects[0], lead_);
		for (size_t i = 0; i < radar_pc->objects.size(); ++i) {
			float dist = apsrc_lead_vehicle_detection::radar_KFobj_dist_func(radar_pc->objects[i], lead_);
			if (dist <= min_dist){
				min_dist = dist;
				closes_object_loc_id = i;
			}
		}

    if (min_dist > roi_max_dist_){
      counter_++;
      return;
    } else {
      counter_ = 0;
    }

		ApsrcLeadVehicleDetectionNl::KF_update_radar(radar_pc->objects[closes_object_loc_id].pose.pose.position.x,
																								 radar_pc->objects[closes_object_loc_id].twist.twist.linear.x,
																								 radar_pc->objects[closes_object_loc_id].pose.pose.position.y,
																								 radar_pc->objects[closes_object_loc_id].twist.twist.linear.y
                                                 );
		
	}
}

void ApsrcLeadVehicleDetectionNl::lidarObjectDetectionCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& lidar_objs)
{
  std::unique_lock<std::mutex> status_lock(read_lock_);
  if (lidar_objs->objects.size() == 0){
    return;
  }
	float min_dist = 0;
	size_t closes_object_loc_id;
	if (lead_.initilized && !lead_.reset) {
		closes_object_loc_id = 0;
    min_dist = apsrc_lead_vehicle_detection::lidar_KFobj_dist_func(lidar_objs->objects[0], lead_, lidar_radar_dist_);
		for (size_t i = 0; i < lidar_objs->objects.size(); ++i){
			if (ApsrcLeadVehicleDetectionNl::emergency_stop_func(lidar_objs->objects[i])){
				stop_now_ = true;
				return;
			}
			if (lidar_objs->objects[i].label == "car"){
				float dist = apsrc_lead_vehicle_detection::lidar_KFobj_dist_func(lidar_objs->objects[i], lead_, lidar_radar_dist_);
				if (dist <= min_dist){
					min_dist = dist;
					closes_object_loc_id = i;
				}
			}
		}

    if (min_dist > roi_max_dist_){
      counter_++;
      return;
    }

    if (ApsrcLeadVehicleDetectionNl::pointsCallback(lidar_objs->objects[closes_object_loc_id].pointcloud)){
      ApsrcLeadVehicleDetectionNl::KF_update_lidar(points_[0].x - lidar_radar_dist_, points_[0].y); 
    } else {
      ApsrcLeadVehicleDetectionNl::KF_update_lidar(lidar_objs->objects[closes_object_loc_id].pose.position.x - lidar_radar_dist_,
                                                  lidar_objs->objects[closes_object_loc_id].pose.position.y);
    }
       
	} else {
    apsrc_lead_vehicle_detection::KFpoint fresh;
    lead_ = fresh;
		autoware_msgs::DetectedObject ref = {};
		min_dist = apsrc_lead_vehicle_detection::lidar_objs_dist_func(lidar_objs->objects[0], ref);
		closes_object_loc_id = 0;
		for (size_t i = 0; i < lidar_objs->objects.size(); ++i){
			float dist = apsrc_lead_vehicle_detection::lidar_objs_dist_func(lidar_objs->objects[i], ref);
			if (std::abs(lidar_objs->objects[i].pose.position.y) <= roi_max_lat_ && lidar_objs->objects[i].pose.position.x >= roi_min_lng_ + lidar_radar_dist_ && dist <= min_dist){
				if (ApsrcLeadVehicleDetectionNl::emergency_stop_func(lidar_objs->objects[i])){
					stop_now_ = true;
					return;
				}
				min_dist = dist;
				closes_object_loc_id = i;
			}
		}
		ApsrcLeadVehicleDetectionNl::trackInit(lidar_objs->objects[closes_object_loc_id]);
	}
}

void ApsrcLeadVehicleDetectionNl::timerCallBack()
{
  apsrc_msgs::LeadVehicle msg;
  msg.emergency_stop = stop_now_;
  msg.header.frame_id = "radar_fc";
  msg.header.stamp = ros::Time::now();
  msg.lead_detected = lead_.initilized && counter_ < 10;
  if (msg.lead_detected && current_velocity_ < 1){
    msg.lead_detected = accel_ < 0.05 ;
  }

  if (counter_ >= 10){
    lead_.reset = true;
  }

  if (msg.lead_detected){
		ApsrcLeadVehicleDetectionNl::KF_update_no_observation();
    msg.range = std::sqrt(lead_.X(0) * lead_.X(0) + lead_.X(2) * lead_.X(2));
    msg.speed = std::sqrt((current_velocity_ + lead_.X(1)) * (current_velocity_ + lead_.X(1)) + lead_.X(3) * lead_.X(3)) ;
    if (msg.speed < current_velocity_ * min_lead_ego_speed_rate_){
      msg.emergency_stop = true;
    }
    msg.speed_mph = msg.speed * 2.23694;
    msg.azimuth_score = static_cast<uint8_t>(lead_.X(0)/msg.range*100);
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
    text.header.stamp = msg.header.stamp;
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
      marker.pose.position.x = lead_.X(0);
      marker.pose.position.y = lead_.X(2);
      marker.action = visualization_msgs::Marker::ADD;     
      text.pose.position.x = lead_.X(0);
      text.pose.position.y = lead_.X(2);
      std::string stop_str = msg.emergency_stop ? "true" : "false";
      text.text = "speed: " + std::to_string(msg.speed) + "m/s\n" +
      std::to_string(msg.speed_mph) + "mph\n" + 
      "dist: "  + std::to_string(msg.range) + "m\n" + 
      "CoVar: " + std::to_string(msg.covar_score) + "%\n" +
      "AzSc: " + std::to_string(msg.azimuth_score);
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
  counter_++;
}

void ApsrcLeadVehicleDetectionNl::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity)
{
  accel_ = std::round(std::abs(current_velocity->twist.linear.x)) - static_cast<double>(current_velocity_);
  current_velocity_ = static_cast<uint16_t>(std::round(std::abs(current_velocity->twist.linear.x)));
}

bool ApsrcLeadVehicleDetectionNl::emergency_stop_func(autoware_msgs::DetectedObject obj)
{
  if (obj.pose.position.x > lidar_radar_dist_ && obj.pose.position.x < current_velocity_ * min_time_gap_ + lidar_radar_dist_ 
      && std::abs(obj.pose.position.y) <= 2){
    return true;
  }
  return false;
}

void ApsrcLeadVehicleDetectionNl::trackInit(autoware_msgs::DetectedObject obj)
{
	if (lead_.initilized){
		return;
	}

  std::unique_lock<std::mutex> status_lock(kf_lock_);
  if (ApsrcLeadVehicleDetectionNl::pointsCallback(obj.pointcloud)){
    lead_.X(0) = points_[0].x - lidar_radar_dist_;
    lead_.X(2) = points_[0].y;
  } else {
    lead_.X(0) = obj.pose.position.x - lidar_radar_dist_;
    lead_.X(2) = obj.pose.position.y;
  }
  lead_.stamp = ros::Time::now();
  lead_.initilized = true;
  return;
}

void ApsrcLeadVehicleDetectionNl::KF_update_no_observation()
{
  std::unique_lock<std::mutex> status_lock(kf_lock_);
  float dT = (ros::Time::now() - lead_.stamp).toSec();
  lead_.stamp = ros::Time::now();
  lead_.F(0, 1) = dT;
  lead_.F(2, 3) = dT;
  lead_.X = lead_.F * lead_.X;
  lead_.P = lead_.F * lead_.P * lead_.F.transpose() + Eigen::Matrix4f::Identity();
}

void ApsrcLeadVehicleDetectionNl::KF_update_lidar(float x, float y)
{
  ApsrcLeadVehicleDetectionNl::KF_update_no_observation();
  try {
    std::unique_lock<std::mutex> status_lock(kf_lock_);
    Eigen::Matrix<float, 4, 1> z = {x, lead_.X(1), y, lead_.X(3)};
    Eigen::Matrix<float, 4, 1> Y = z - lead_.X;
    Eigen::Matrix<float, 4, 4> S = lead_.P + Y * Y.transpose() * lidar_x_gain_;
    Eigen::Matrix<float, 4, 4> K = lead_.P * S.inverse();
    lead_.X = lead_.X + K * Y;
    lead_.P = (Eigen::Matrix4f::Identity() - K) * lead_.P;
  } catch (const std::exception& e) {
    return;
  }
  return;
}

void ApsrcLeadVehicleDetectionNl::KF_update_radar(float x, float dx, float y, float dy)
{
  ApsrcLeadVehicleDetectionNl::KF_update_no_observation();
  try {
    std::unique_lock<std::mutex> status_lock(kf_lock_);
    Eigen::Matrix<float, 4, 1> z = {x, dx, y, dy};
    Eigen::Matrix<float, 4, 1> Y = z - lead_.X;
    Eigen::Matrix<float, 4, 4> S = lead_.P + Y * Y.transpose();
    Eigen::Matrix<float, 4, 4> K = lead_.P * S.inverse();
    lead_.X = lead_.X + K * Y;
    lead_.P = lead_.P * (Eigen::Matrix4f::Identity() - K);
  } catch (const std::exception& e) {
    return;
  }
  return;
}

bool ApsrcLeadVehicleDetectionNl::pointsCallback(const sensor_msgs::PointCloud2_<std::allocator<void>> &msg)
{
  pcl::PointCloud<pcl::PointXYZ> sub_points;
  pcl::fromROSMsg(msg, sub_points);

  points_.clear();
  if (sub_points.size() > 0){
    float argmin = 0;
    for (size_t i = 0; i < sub_points.size(); ++ i){
      if (sub_points[i].x < sub_points[argmin].x){
        argmin = i;
      }
    }
    points_.push_back(sub_points[argmin]);
    return true;
  }
  return false;
}
}  // namespace apsrc_lead_vehicle_detection
PLUGINLIB_EXPORT_CLASS(apsrc_lead_vehicle_detection::ApsrcLeadVehicleDetectionNl,
                       nodelet::Nodelet);
