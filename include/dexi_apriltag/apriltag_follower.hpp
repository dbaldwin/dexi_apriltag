#ifndef DEXI_APRILTAG__APRILTAG_FOLLOWER_HPP_
#define DEXI_APRILTAG__APRILTAG_FOLLOWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <atomic>
#include <memory>
#include <cmath>

class AprilTagFollower : public rclcpp::Node
{
public:
    AprilTagFollower(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~AprilTagFollower();

private:
    // QoS profile
    rclcpp::QoS qos_profile_;

    // ROS publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr visual_offset_publisher_;

    // ROS subscribers
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_pos_subscriber_;

    // Timer for control loop
    rclcpp::TimerBase::SharedPtr control_timer_;

    // TF2 for looking up tag transforms
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // State variables
    double drone_x_{0.0}, drone_y_{0.0}, drone_z_{0.0}, drone_heading_{0.0};
    bool drone_position_valid_{false};
    std::string camera_frame_id_{"camera"};
    std::string current_tag_frame_{""};

    // Camera parameters
    double camera_fx_{0.0}, camera_fy_{0.0};  // Focal lengths
    double camera_cx_{0.0}, camera_cy_{0.0};  // Principal point
    double camera_width_{640.0}, camera_height_{480.0};
    bool camera_info_received_{false};

    // AprilTag detection state
    std::atomic<bool> tag_detected_{false};
    double tag_pixel_x_{0.0};      // Tag center in image coordinates (pixels)
    double tag_pixel_y_{0.0};
    double tag_distance_{0.0};     // Distance to tag in meters
    double tag_yaw_{0.0};          // Tag yaw angle in camera frame (radians)
    rclcpp::Time last_detection_time_;
    double detection_timeout_{10.0}; // Seconds before considering tag lost
    double initial_altitude_{0.0}; // Altitude to maintain when following
    bool altitude_initialized_{false};
    double target_yaw_{0.0};       // Target yaw to maintain when following
    bool yaw_initialized_{false};

    // Control parameters
    bool following_enabled_{false};
    double target_altitude_{2.0};     // Target altitude above ground (meters, positive = up)
    double proportional_gain_xy_{0.5}; // P gain for horizontal movement
    double proportional_gain_z_{0.3};  // P gain for altitude
    double max_velocity_xy_{0.5};     // Max horizontal velocity (m/s)
    double max_velocity_z_{0.3};      // Max vertical velocity (m/s)
    double deadband_pixels_{20.0};    // Pixel deadband for considering tag centered
    int target_tag_id_{0};            // Which tag ID to follow (0 = any tag)

    // Methods
    void initializePublishers();
    void initializeSubscribers();
    void initializeTimer();

    // Callbacks
    void apriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void vehicleLocalPosCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void controlLoop();

    // Control methods
    void computeAndPublishSetpoint();
    bool isTagCentered();

    // Utility methods
    uint64_t getTimestamp();
};

#endif // DEXI_APRILTAG__APRILTAG_FOLLOWER_HPP_
