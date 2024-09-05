#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "videocapture.hpp"
#include "sensorcapture.hpp"
#include <opencv2/opencv.hpp>

class ZedCameraNode : public rclcpp::Node
{
public:
    ZedCameraNode()
        : Node("zed_camera_node")
    {
        // Initialize publishers
        left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/left/image_raw", 10);
        right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/right/image_raw", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

        // Initialize the camera
        if (!video_capture_.initializeVideo()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize video capture");
            return;
        }

        // Initialize sensor capture
        std::vector<int> devs = sensor_capture_.getDeviceList();
        if (!devs.empty()) {
            if (!sensor_capture_.initializeSensors(devs[0])) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize sensor capture");
                return;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "No devices found for sensor capture");
            return;
        }

        // Create a timer to capture and publish data at 10Hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ZedCameraNode::captureAndPublish, this));
    }

private:
    void captureAndPublish()
    {
        // Capture and publish IMU data
        sl_oc::sensors::data::Imu imu_data = sensor_capture_.getLastIMUData(5000);
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.linear_acceleration.x = imu_data.aX;
        imu_msg.linear_acceleration.y = imu_data.aY;
        imu_msg.linear_acceleration.z = imu_data.aZ;
        imu_msg.angular_velocity.x = imu_data.gX;
        imu_msg.angular_velocity.y = imu_data.gY;
        imu_msg.angular_velocity.z = imu_data.gZ;
        imu_pub_->publish(imu_msg);

        // Log the IMU data
        RCLCPP_INFO(this->get_logger(), "IMU data: aX=%f, aY=%f, aZ=%f, gX=%f, gY=%f, gZ=%f",
                    imu_data.aX, imu_data.aY, imu_data.aZ, imu_data.gX, imu_data.gY, imu_data.gZ);

        // Capture and publish video frames
        cv::Mat frameYUV, frameBGR, left_raw, right_raw;
        const sl_oc::video::Frame frame = video_capture_.getLastFrame();

        // Check if frame is valid before processing
        if (frame.data != nullptr && frame.width > 0 && frame.height > 0) {
            frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

            left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
            right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));

            // Resize images to 1104x621 (compressed format)
            cv::Mat left_resized, right_resized;
            cv::resize(left_raw, left_resized, cv::Size(752, 480));
            cv::resize(right_raw, right_resized, cv::Size(752, 480));

            // Log left_resized and right_resized frame dimensions
            RCLCPP_INFO(this->get_logger(), "Left resized frame data: width=%d, height=%d", left_resized.cols, left_resized.rows);
            RCLCPP_INFO(this->get_logger(), "Right resized frame data: width=%d, height=%d", right_resized.cols, right_resized.rows);

            // Convert cv::Mat to ROS Image message
            sensor_msgs::msg::Image::SharedPtr left_resized_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_resized).toImageMsg();
            sensor_msgs::msg::Image::SharedPtr right_resized_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_resized).toImageMsg();
            
            // Set timestamps to now

            left_resized_msg->header.stamp = this->now();
            right_resized_msg->header.stamp = this->now();

            // Publish the resized image
            RCLCPP_INFO(this->get_logger(), "Publishing left and right images");
            left_image_pub_->publish(*left_resized_msg);
            right_image_pub_->publish(*right_resized_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid frame data or dimensions: width=%d, height=%d", frame.width, frame.height);
        }
    }

    // Private members
    sl_oc::video::VideoCapture video_capture_;
    sl_oc::sensors::SensorCapture sensor_capture_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedCameraNode>());
    rclcpp::shutdown();
    return 0;
}
