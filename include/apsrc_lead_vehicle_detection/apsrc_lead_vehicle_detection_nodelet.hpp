#ifndef ApsrcLeadVehicleDetectionNl_H
#define ApsrcLeadVehicleDetectionNl_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <thread>
#include <mutex>
#include <Eigen/Dense>


#include <geometry_msgs/TwistStamped.h>
#include <derived_object_msgs/ObjectWithCovarianceArray.h>
#include <derived_object_msgs/ObjectWithCovariance.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>

#include <apsrc_msgs/LaedVehicle.h>
#include <visualization_msgs/Marker.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace apsrc_lead_vehicle_detection
{
struct KFpoint
{
  ros::Time stamp;
  Eigen::Matrix<float, 4, 1> X = {0, 0, 0, 0};
  Eigen::Matrix<float, 4, 4> F = Eigen::Matrix4f::Identity();
  Eigen::Matrix<float, 4, 4> P = Eigen::Matrix4f::Identity();
  bool tracked = false;
  bool initilized = false;
};

class ApsrcLeadVehicleDetectionNl : public nodelet::Nodelet
{
public:
  ApsrcLeadVehicleDetectionNl();
  ~ApsrcLeadVehicleDetectionNl();

private:
  // Init
  virtual void onInit();
  void loadParams();
  std::mutex point_mtx_;

  // Subscriber callbacks
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity);
  void radarPointCloudCallback(const derived_object_msgs::ObjectWithCovarianceArray::ConstPtr& radar_pc);
  void lidarObjectDetectionCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& lidar_objs);
  void timerCallBack();

  // Nodehandles
  ros::NodeHandle nh_, pnh_;

  // Publishers
  ros::Publisher lead_vehicle_pub_;
  ros::Publisher marker_pub_;
  ros::Timer timer_;

  // Subscribers
  ros::Subscriber current_velocity_sub_;
  ros::Subscriber radar_point_cloud_sub_;
  ros::Subscriber lidar_detection_sub_;

  // Current velocity of the vehicle (mm/s)
  uint16_t current_velocity_ = 0;
  ros::Time current_velocity_rcvd_time_;

  // Lead vehicle
  KFpoint lead_;

  // Radar read
  std::string radar_pc_topic_name_;
  derived_object_msgs::ObjectWithCovariance radar_lead_;
  bool radar_lead_found_ = true;
  int8_t confidence_ = 0;

  // Lidar object
  std::string lidar_objs_topic_name_;
  autoware_msgs::DetectedObject lidar_lead_;
  autoware_msgs::DetectedObject lidar_lead_t_;
  bool lidar_lead_found_ = false;

  // ROI / Search
  double roi_max_lat_ = 2.0;
  double roi_min_lng_ = 1.0;
  double roi_max_dist_ = 5.0;
  double min_time_gap_ = 0.9;
  double min_lead_ego_speed_rate_ = 0.5;
  bool stop_now_ = false;

  // Internal
  double lidar_radar_dist_ = 3.588; // [m] Pacifica
  bool emergency_stop_func(autoware_msgs::DetectedObject obj);
  bool visualization_;

};

float radar_objs_dist_func(derived_object_msgs::ObjectWithCovariance pointA, derived_object_msgs::ObjectWithCovariance pointB)
{
  float dx = pointA.pose.pose.position.x - pointB.pose.pose.position.x;
  float dy = pointA.pose.pose.position.y - pointB.pose.pose.position.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  return dist;
};

float lidar_objs_dist_func(autoware_msgs::DetectedObject objA, autoware_msgs::DetectedObject objB)
{
  float dx = objA.pose.position.x - objB.pose.position.x;
  float dy = objA.pose.position.y - objB.pose.position.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  return dist;
};

float lidar_radar_dist_func(autoware_msgs::DetectedObject objA, derived_object_msgs::ObjectWithCovariance pointB)
{
  float dx = objA.pose.position.x - pointB.pose.pose.position.x;
  float dy = objA.pose.position.y - pointB.pose.pose.position.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  return dist;
};

float KF_update_no_observation(KFpoint obj)
{
  ros::Time now = ros::Time::now();
  float dT = (now - obj.stamp).toSec();
  obj.stamp = now;
  obj.F(0, 1) = dT;
  obj.F(2, 3) = dT;
  obj.X = obj.F * obj.X;
  obj.P = obj.F * obj.P * obj.F.transpose();
  return obj.P.determinant();
};

float KF_update_lidar(KFpoint obj, float x, float y)
{
  KF_update_no_observation(obj);
  try {
    Eigen::Matrix<float, 4, 1> z = {x, obj.X(0, 1), y, obj.X(0, 3)};
    Eigen::Matrix<float, 4, 1> Y = z - obj.X;
    Eigen::Matrix<float, 4, 4> S = obj.P + Y * Y.transpose();
    Eigen::Matrix<float, 4, 4> K = obj.P * S.inverse();
    obj.X = obj.X + K * Y;
    obj.P = obj.P * (Eigen::Matrix4f::Identity() - K); 
  } catch (const std::exception& e) {
    return -1;
  }
  return obj.P.determinant();
};

float KF_update_radar(KFpoint obj, float x, float dx, float y, float dy)
{
  KF_update_no_observation(obj);
  try {
    Eigen::Matrix<float, 4, 1> z = {x, dx, y, dy};
    Eigen::Matrix<float, 4, 1> Y = z - obj.X;
    Eigen::Matrix<float, 4, 4> S = obj.P + Y * Y.transpose();
    Eigen::Matrix<float, 4, 4> K = obj.P * S.inverse();
    obj.X = obj.X + K * Y;
    obj.P = obj.P * (Eigen::Matrix4f::Identity() - K); 
  } catch (const std::exception& e) {
    return -1;
  }
  return obj.P.determinant();
};

}  // namespace apsrc_lead_vehicle_detection
#endif  // ApsrcLeadVehicleDetectionNl_H
