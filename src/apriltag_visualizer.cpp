#include "dexi_apriltag/apriltag_visualizer.hpp"

AprilTagVisualizer::AprilTagVisualizer(const rclcpp::NodeOptions &options)
: Node("apriltag_visualizer", options)
{
    // Declare parameters
    this->declare_parameter("publish_raw", true);
    this->declare_parameter("publish_compressed", true);

    // Get parameters
    publish_raw_ = this->get_parameter("publish_raw").as_bool();
    publish_compressed_ = this->get_parameter("publish_compressed").as_bool();

    // Initialize publishers
    if (publish_raw_) {
        debug_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
            "apriltag_debug_image", 10);
    }

    if (publish_compressed_) {
        debug_image_compressed_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "apriltag_debug_image/compressed", 10);
    }

    // Initialize subscribers
    image_subscriber_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        "/cam0/image_raw/compressed",
        10,
        std::bind(&AprilTagVisualizer::imageCallback, this, std::placeholders::_1));

    visual_offset_subscriber_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/dexi/apriltag_visual_offset",
        10,
        std::bind(&AprilTagVisualizer::visualOffsetCallback, this, std::placeholders::_1));

    apriltag_subscriber_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/apriltag_detections",
        10,
        std::bind(&AprilTagVisualizer::apriltagCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "AprilTag Visualizer initialized");
}

AprilTagVisualizer::~AprilTagVisualizer()
{
}

void AprilTagVisualizer::imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try {
        // Decode compressed image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        latest_image_ = cv_ptr->image.clone();
        image_received_ = true;

        // Update image center
        image_center_x_ = latest_image_.cols / 2.0;
        image_center_y_ = latest_image_.rows / 2.0;

        // Draw visualization whenever we get a new image
        drawVisualization();

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void AprilTagVisualizer::visualOffsetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    offset_x_pixels_ = msg->point.x;
    offset_y_pixels_ = msg->point.y;
    tag_distance_ = msg->point.z;
    offset_received_ = true;
}

void AprilTagVisualizer::apriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    if (msg->detections.empty()) {
        tag_detected_ = false;
        tag_corners_.clear();
        return;
    }

    // Use the first detection for visualization
    const auto& detection = msg->detections[0];
    tag_detected_ = true;
    tag_center_x_ = detection.centre.x;
    tag_center_y_ = detection.centre.y;

    // Store tag corners for drawing the tag outline
    tag_corners_.clear();
    for (const auto& corner : detection.corners) {
        tag_corners_.push_back(cv::Point2d(corner.x, corner.y));
    }
}

void AprilTagVisualizer::drawVisualization()
{
    if (!image_received_) {
        return;
    }

    // Make a copy of the image to draw on
    cv::Mat viz_image = latest_image_.clone();

    // Define colors
    cv::Scalar color_center(0, 255, 0);      // Green for image center
    cv::Scalar color_tag(0, 0, 255);         // Red for tag
    cv::Scalar color_offset(255, 255, 0);    // Cyan for offset line
    cv::Scalar color_text(255, 255, 255);    // White for text

    // Draw image center crosshair
    int crosshair_size = 20;
    cv::Point center_pt(static_cast<int>(image_center_x_),
                       static_cast<int>(image_center_y_));
    cv::line(viz_image,
             center_pt - cv::Point(crosshair_size, 0),
             center_pt + cv::Point(crosshair_size, 0),
             color_center, 2);
    cv::line(viz_image,
             center_pt - cv::Point(0, crosshair_size),
             center_pt + cv::Point(0, crosshair_size),
             color_center, 2);
    cv::circle(viz_image, center_pt, 5, color_center, 2);

    // Draw tag detection if available
    if (tag_detected_) {
        cv::Point tag_pt(static_cast<int>(tag_center_x_),
                        static_cast<int>(tag_center_y_));

        // Draw tag outline from corners
        if (tag_corners_.size() == 4) {
            std::vector<cv::Point> corners_int;
            for (const auto& corner : tag_corners_) {
                corners_int.push_back(cv::Point(static_cast<int>(corner.x),
                                               static_cast<int>(corner.y)));
            }
            for (size_t i = 0; i < 4; i++) {
                cv::line(viz_image, corners_int[i], corners_int[(i+1)%4],
                        color_tag, 2);
            }
        }

        // Draw tag center
        cv::circle(viz_image, tag_pt, 8, color_tag, -1);

        // Draw offset line from center to tag
        cv::line(viz_image, center_pt, tag_pt, color_offset, 2);
        cv::arrowedLine(viz_image, center_pt, tag_pt, color_offset, 3, 8, 0, 0.3);

        // Draw offset values as text
        if (offset_received_) {
            char text[128];
            snprintf(text, sizeof(text), "Offset: (%.0f, %.0f) px",
                    offset_x_pixels_, offset_y_pixels_);
            cv::putText(viz_image, text, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, color_text, 2);

            snprintf(text, sizeof(text), "Distance: %.2f m", tag_distance_);
            cv::putText(viz_image, text, cv::Point(10, 60),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, color_text, 2);

            // Draw offset magnitude as text near the line
            double offset_magnitude = std::sqrt(offset_x_pixels_ * offset_x_pixels_ +
                                               offset_y_pixels_ * offset_y_pixels_);
            snprintf(text, sizeof(text), "%.0f px", offset_magnitude);
            cv::Point mid_pt((center_pt.x + tag_pt.x) / 2,
                           (center_pt.y + tag_pt.y) / 2);
            cv::putText(viz_image, text, mid_pt,
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color_offset, 2);
        }
    } else {
        // Draw "No tag detected" message
        cv::putText(viz_image, "No AprilTag detected", cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    }

    // Publish raw image if enabled
    if (publish_raw_ && debug_image_publisher_) {
        auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(),
                                         sensor_msgs::image_encodings::BGR8,
                                         viz_image).toImageMsg();
        out_msg->header.stamp = this->now();
        out_msg->header.frame_id = "camera";
        debug_image_publisher_->publish(*out_msg);
    }

    // Publish compressed image if enabled
    if (publish_compressed_ && debug_image_compressed_publisher_) {
        sensor_msgs::msg::CompressedImage compressed_msg;
        compressed_msg.header.stamp = this->now();
        compressed_msg.header.frame_id = "camera";
        compressed_msg.format = "jpeg";

        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        cv::imencode(".jpg", viz_image, buffer, params);
        compressed_msg.data = buffer;

        debug_image_compressed_publisher_->publish(compressed_msg);
    }
}

// Main function
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
