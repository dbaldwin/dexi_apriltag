# dexi_apriltag

AprilTag detection and following package for DEXI drones.

## Overview

This package provides AprilTag-based visual tracking and following capabilities for PX4-based drones. It has been separated from the main `dexi_offboard` package to isolate computer vision dependencies and enable independent development.

## Nodes

### apriltag_follower

Visual servo control node that commands the drone to follow detected AprilTags.

**Published Topics:**
- `/fmu/in/offboard_control_mode` - Offboard control mode heartbeat
- `/fmu/in/trajectory_setpoint` - Position setpoints for PX4
- `apriltag_visual_offset` - Debug topic showing pixel offset from image center

**Subscribed Topics:**
- `/detections` - AprilTag detections from apriltag_ros
- `/camera_info` - Camera calibration parameters
- `/fmu/out/vehicle_local_position` - Current drone position

**Parameters:**
- `following_enabled` (bool, default: false) - Enable/disable active following
- `target_altitude` (double, default: 2.0) - Target altitude in meters
- `proportional_gain_xy` (double, default: 0.5) - P gain for XY movement
- `proportional_gain_z` (double, default: 0.3) - P gain for altitude
- `max_velocity_xy` (double, default: 0.5) - Max horizontal velocity (m/s)
- `max_velocity_z` (double, default: 0.3) - Max vertical velocity (m/s)
- `deadband_pixels` (double, default: 20.0) - Centering tolerance in pixels
- `target_tag_id` (int, default: 0) - Specific tag ID to follow (0 = any tag)

### apriltag_visualizer

Visualization node that overlays AprilTag detection information on camera images.

**Published Topics:**
- `apriltag_debug_image` - Raw debug image with overlays
- `apriltag_debug_image/compressed` - Compressed debug image

**Subscribed Topics:**
- `/cam0/image_raw/compressed` - Camera image stream
- `/dexi/apriltag_visual_offset` - Follower offset information
- `/apriltag_detections` - AprilTag detection results

**Parameters:**
- `publish_raw` (bool, default: true) - Publish uncompressed debug images
- `publish_compressed` (bool, default: true) - Publish compressed debug images

## Launch Files

### apriltag_follow.launch.py

Launches both the follower and visualizer nodes with configurable parameters.

**Usage:**
```bash
ros2 launch dexi_apriltag apriltag_follow.launch.py following_enabled:=true
```

## Dependencies

This package requires:
- `cv_bridge` - OpenCV-ROS bridge
- `image_transport` - Image transport plugins
- `apriltag_msgs` - AprilTag detection messages
- `px4_msgs` - PX4 autopilot messages
- Standard ROS 2 packages (rclcpp, tf2, geometry_msgs, sensor_msgs)

## Architecture Notes

The `apriltag_follower` node directly publishes PX4 control commands and operates independently of the `dexi_offboard` package. This allows it to control the drone without going through the offboard manager, enabling dedicated visual servoing control loops.
