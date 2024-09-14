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
        // Initialize camera calibration parameters
        initializeCalibrationParams();

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
    const int IMAGE_WIDTH = 640;
    const int IMAGE_HEIGHT = 480;

    void initializeCalibrationParams()
    {
        // LEFT camera intrinsic matrix (K)
        K_l = (cv::Mat_<double>(3, 3) << 351.553, 0, 332.793, 0, 468.532, 242.539, 0, 0, 1);

        // RIGHT camera intrinsic matrix (K)
        K_r = (cv::Mat_<double>(3, 3) << 351.577, 0, 318.017, 0, 468.509, 248.872, 0, 0, 1);

        // LEFT and RIGHT rectification matrices
        R_l = cv::Mat::eye(3, 3, CV_64F); // Identity matrix
        R_r = cv::Mat::eye(3, 3, CV_64F);

        // LEFT and RIGHT distortion coefficients
        D_l = (cv::Mat_<double>(1, 5) << -0.0406, 0.0095, -0.0006, -0.0001, -0.0047);
        D_r = (cv::Mat_<double>(1, 5) << -0.0414, 0.0094, 0.0001, 0.0001, -0.0046);

        // LEFT and RIGHT projection matrices
        P_l = (cv::Mat_<double>(3, 4) << 351.553, 0, 332.793, 0, 0, 468.532, 242.539, 0, 0, 0, 1, 0);
        P_r = (cv::Mat_<double>(3, 4) << 351.577, 0, 318.017, 120, 0, 468.509, 248.872, 0, 0, 0, 1, 0);

        // Create rectification maps
        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l(cv::Range(0, 3), cv::Range(0, 3)), cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r(cv::Range(0, 3), cv::Range(0, 3)), cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_32F, M1r, M2r);
    }

    void captureAndPublish()
    {
        captureAndPublishIMU();
        captureAndPublishImages();
    }

    void captureAndPublishIMU()
    {
        // Capture and publish IMU data
        sl_oc::sensors::data::Imu imu_data = sensor_capture_.getLastIMUData(5000);
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_frame"; // Make sure to set the frame ID properly
        imu_msg.linear_acceleration.x = imu_data.aX;
        imu_msg.linear_acceleration.y = imu_data.aY;
        imu_msg.linear_acceleration.z = imu_data.aZ;
        imu_msg.angular_velocity.x = imu_data.gX;
        imu_msg.angular_velocity.y = imu_data.gY;
        imu_msg.angular_velocity.z = imu_data.gZ;
        imu_pub_->publish(imu_msg);

        RCLCPP_DEBUG(this->get_logger(), "IMU data: aX=%f, aY=%f, aZ=%f, gX=%f, gY=%f, gZ=%f",
                     imu_data.aX, imu_data.aY, imu_data.aZ, imu_data.gX, imu_data.gY, imu_data.gZ);
    }

    void captureAndPublishImages()
    {
        cv::Mat frameYUV, frameBGR, left_raw, right_raw;
        const sl_oc::video::Frame frame = video_capture_.getLastFrame();

        if (frame.data != nullptr && frame.width > 0 && frame.height > 0)
        {
            // Convert and split the stereo image
            frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);
            left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
            right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));

            // Resize the images
            cv::Mat left_resized, right_resized;
            cv::resize(left_raw, left_resized, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));
            cv::resize(right_raw, right_resized, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));

            // Rectify the images
            cv::Mat left_rectified, right_rectified;
            cv::remap(left_resized, left_rectified, M1l, M2l, cv::INTER_LINEAR);
            cv::remap(right_resized, right_rectified, M1r, M2r, cv::INTER_LINEAR);

            // Publish the rectified images
            publishImage(left_rectified, left_image_pub_);
            publishImage(right_rectified, right_image_pub_);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid frame data or dimensions: width=%d, height=%d", frame.width, frame.height);
        }
    }

    void publishImage(const cv::Mat &image, const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub)
    {
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        img_msg->header.stamp = this->now();
        img_msg->header.frame_id = "camera_frame"; // Set frame ID properly
        pub->publish(*img_msg);
    }

    // Private members
    sl_oc::video::VideoCapture video_capture_;
    sl_oc::sensors::SensorCapture sensor_capture_;
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
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
