# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "apsrc_msgs;autoware_msgs;geometry_msgs;nodelet;roscpp;roslib;roslint;std_msgs;radar_msgs;derived_object_msgs;tf2_ros".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lapsrc_lead_vehicle_detection_nodelets".split(';') if "-lapsrc_lead_vehicle_detection_nodelets" != "" else []
PROJECT_NAME = "apsrc_lead_vehicle_detection"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "1.0.0"
