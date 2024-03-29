#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/loader.h>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apsrc_lead_vehicle_detection_node");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "apsrc_lead_vehicle_detection/apsrc_lead_vehicle_detection_nodelet", remap, nargv);
  ros::spin();
  return 0;
}
