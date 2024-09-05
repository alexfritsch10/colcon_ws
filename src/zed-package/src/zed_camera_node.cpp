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
        // Create rectification maps
        const string strSettingsFile = "../ZED2.yaml";

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Wrong path to settings");
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Calibration parameters to rectify stereo are missing!");
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);

        // Initialize publishers
        left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/left/image_rect", 10);
        right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/right/image_rect", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

        // Initialize the camera
        if (!video_capture_.initializeVideo())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize video capture");
            return;
        }

        // Initialize sensor capture
        std::vector<int> devs = sensor_capture_.getDeviceList();
        if (!devs.empty())
        {
            if (!sensor_capture_.initializeSensors(devs[0]))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize sensor capture");
                return;
            }
        }
        else
        {
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
        if (frame.data != nullptr && frame.width > 0 && frame.height > 0)
        {
            frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

            left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
            right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));

            // Resize images to 1104x621 (compressed format)
            cv::Mat left_resized, right_resized;
            cv::resize(left_raw, left_resized, cv::Size(640, 480));
            cv::resize(right_raw, right_resized, cv::Size(640, 480));

            // Log left_resized and right_resized frame dimensions
            RCLCPP_INFO(this->get_logger(), "Left resized frame data: width=%d, height=%d", left_resized.cols, left_resized.rows);
            RCLCPP_INFO(this->get_logger(), "Right resized frame data: width=%d, height=%d", right_resized.cols, right_resized.rows);

            // Rectify the resized images using remap
            cv::Mat left_rectified, right_rectified;
            cv::remap(left_resized, left_rectified, M1l, M2l, cv::INTER_LINEAR);
            cv::remap(right_resized, right_rectified, M1r, M2r, cv::INTER_LINEAR);

            // Convert rectified images to ROS Image messages
            sensor_msgs::msg::Image::SharedPtr left_rectified_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_rectified).toImageMsg();
            sensor_msgs::msg::Image::SharedPtr right_rectified_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_rectified).toImageMsg();

            // Set timestamps for the rectified images
            left_rectified_msg->header.stamp = this->now();
            right_rectified_msg->header.stamp = this->now();

            // Publish the rectified images
            RCLCPP_INFO(this->get_logger(), "Publishing rectified left and right images");
            left_image_pub_->publish(*left_rectified_msg);
            right_image_pub_->publish(*right_rectified_msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid frame data or dimensions: width=%d, height=%d", frame.width, frame.height);
        }
    }

    // Private members
    sl_oc::video::VideoCapture video_capture_;
    sl_oc::sensors::SensorCapture sensor_capture_;

    cv::Mat M1l, M2l, M1r, M2r;

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
