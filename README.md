# Lead Vehicle Detection Nodelet

This package provides a nodelet for lead vehicle detection using radar and lidar data in the APSRC system.

## Overview

The lead vehicle detection nodelet subscribes to radar and lidar data, processes it to detect lead vehicles, and publishes the detected lead vehicle information. It utilizes both radar and lidar data to enhance detection accuracy.

## Installation

Clone this repository into your catkin workspace and build it using catkin_make:

```bash
cd catkin_ws/src
git clone <repository_url>
cd ..
catkin_make
```

## Dependencies

This package depends on the following ROS packages:
- `apsrc_msgs`
- `derived_object_msgs`
- `autoware_msgs`
- `visualization_msgs`
- `geometry_msgs`
- `nodelet`

Make sure these packages are installed in your ROS environment.

## Usage

Launch the nodelet using a launch file or through a nodelet manager.

```xml
<launch>
    <node pkg="nodelet" type="nodelet" name="lead_vehicle_detection_nodelet" args="manager" output="screen"/>

    <node pkg="nodelet" type="nodelet" name="apsrc_lead_vehicle_detection" args="load apsrc_lead_vehicle_detection/ApsrcLeadVehicleDetectionNl" output="screen">
        <remap from="current_velocity" to="/your/current_velocity_topic"/>
    </node>
</launch>
```

## Parameters

The nodelet can be configured using the following parameters:

- `radar_pc_topic`: Radar point cloud topic name (default: "/radar_fc/as_tx/objects")
- `lidar_objs_topic`: Lidar object topic name (default: "/detection/lidar_objects")
- `roi_max_lat`: Maximum lateral distance considered for lead vehicle detection (default: 2.0 meters)
- `roi_min_lng`: Minimum longitudinal distance considered for lead vehicle detection (default: 1.0 meter)
- `roi_max_dist`: Maximum distance considered for lead vehicle detection (default: 2.0 meters)
- `min_time_gap`: Minimum time gap between the lead vehicle and ego vehicle (default: 2.0 seconds)
- `min_lead_ego_speed_rate`: Minimum lead-to-ego vehicle speed ratio for considering emergency stop (default: 2.0)
- `visualization`: Enable visualization markers (default: true)

## Published Topics

- `/lead_vehicle/track`: Detected lead vehicle information.
- `/lead_vehicle/markers`: Visualization markers for detected lead vehicle.

## Subscribed Topics

- `/current_velocity`: Current velocity information.
- `/radar_fc/as_tx/objects`: Radar point cloud data.
- `/detection/lidar_objects`: Lidar object data.

## Nodelet API

### Classes

- `ApsrcLeadVehicleDetectionNl`: Main class implementing lead vehicle detection.

### Methods

- `onInit()`: Initializes the nodelet.
- `loadParams()`: Loads parameters from ROS parameter server.
- `radarPointCloudCallback()`: Callback function for radar point cloud data.
- `lidarObjectDetectionCallback()`: Callback function for lidar object data.
- `velocityCallback()`: Callback function for current velocity data.
- `emergency_stop_func()`: Checks if emergency stop is required based on detected objects.

## Author

This package is authored by Mojtaba Bahramgiri @APSRC. 
