#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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
        // initializeCameraInfoMsgs();

        // Initialize publishers
        left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/left/image_rect", 10);
        right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/right/image_rect", 10);
        left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/left/camera_info", 10);
        right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/right/camera_info", 10);
        // imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

        // Initialize the camera
        if (!video_capture_.initializeVideo())
        {
            RCLCPP_INFO(this->get_logger(), "Failed to initialize video capture");
            return;
        }

        // Initialize sensor capture
        // std::vector<int> devs = sensor_capture_.getDeviceList();
        // if (!devs.empty())
        // {
        //     if (!sensor_capture_.initializeSensors(devs[0]))
        //     {
        //         RCLCPP_INFO(this->get_logger(), "Failed to initialize sensor capture");
        //         return;
        //     }
        // }
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "No devices found for sensor capture");
        //     return;
        // }

        // Create a timer to capture and publish data at 10Hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ZedCameraNode::captureAndPublish, this));
    }

private:
    const int IMAGE_WIDTH = 1280;
    const int IMAGE_HEIGHT = 720;

    void initializeCalibrationParams()
    {
        // LEFT and RIGHT camera intrinsic matrices (K)
        K_l = (cv::Mat_<double>(3, 3) << 1054.66, 0, 657.69, 0, 1054.35, 361.345, 0, 0, 1);
        K_r = (cv::Mat_<double>(3, 3) << 1054.73, 0, 635.525, 0, 1054.3, 368.3105, 0, 0, 1);

        // LEFT and RIGHT distortion coefficients
        D_l = (cv::Mat_<double>(1, 5) << -0.0406, 0.0095, -0.0006, -0.0001, -0.0047);
        D_r = (cv::Mat_<double>(1, 5) << -0.0414, 0.0094, 0.0001, 0.0001, -0.0046);

        // Rotation and translation between the cameras
        cv::Mat R;
        cv::Mat rvec = (cv::Mat_<double>(3, 1) << -0.0011, 0.0000, -0.0001); // Rotation vector (Rx, Ry, Rz in radians)
        cv::Rodrigues(rvec, R);                                              // Convert the rotation vector into a 3x3 rotation matrix

        cv::Mat T = (cv::Mat_<double>(3, 1) << -120.312, 0.018, -0.7697); // Translation vector (Tx, Ty, Tz)

        // Image size (width and height)
        cv::Size imageSize(IMAGE_WIDTH, IMAGE_HEIGHT);

        // Compute rectification transforms
        cv::stereoRectify(K_l, D_l, K_r, D_r, imageSize, R, T, R_l, R_r, P_l, P_r, Q);

        // Create rectification maps for left and right images
        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l, imageSize, CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r, imageSize, CV_32F, M1r, M2r);
    }

    void initializeCameraInfoMsgs()
    {
        // Left Camera Info Message (https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)
        left_info_msg_.header.frame_id = "camera_link";
        left_info_msg_.width = IMAGE_WIDTH;
        left_info_msg_.height = IMAGE_HEIGHT;
        left_info_msg_.distortion_model = "plumb_bob";
        left_info_msg_.d = {D_l.at<double>(0, 0), D_l.at<double>(0, 1), D_l.at<double>(0, 2), D_l.at<double>(0, 3), D_l.at<double>(0, 4)};
        left_info_msg_.k = {K_l.at<double>(0, 0), K_l.at<double>(0, 1), K_l.at<double>(0, 2),
                            K_l.at<double>(1, 0), K_l.at<double>(1, 1), K_l.at<double>(1, 2),
                            K_l.at<double>(2, 0), K_l.at<double>(2, 1), K_l.at<double>(2, 2)};
        left_info_msg_.r = {R_l.at<double>(0, 0), R_l.at<double>(0, 1), R_l.at<double>(0, 2),
                            R_l.at<double>(1, 0), R_l.at<double>(1, 1), R_l.at<double>(1, 2),
                            R_l.at<double>(2, 0), R_l.at<double>(2, 1), R_l.at<double>(2, 2)};
        left_info_msg_.p = {P_l.at<double>(0, 0), P_l.at<double>(0, 1), P_l.at<double>(0, 2), P_l.at<double>(0, 3),
                            P_l.at<double>(1, 0), P_l.at<double>(1, 1), P_l.at<double>(1, 2), P_l.at<double>(1, 3),
                            P_l.at<double>(2, 0), P_l.at<double>(2, 1), P_l.at<double>(2, 2), P_l.at<double>(2, 3)};

        // Right Camera Info Message
        right_info_msg_.header.frame_id = "camera_link";
        right_info_msg_.width = IMAGE_WIDTH;
        right_info_msg_.height = IMAGE_HEIGHT;
        right_info_msg_.distortion_model = "plumb_bob";
        right_info_msg_.d = {D_r.at<double>(0, 0), D_r.at<double>(0, 1), D_r.at<double>(0, 2), D_r.at<double>(0, 3), D_r.at<double>(0, 4)};
        right_info_msg_.k = {K_r.at<double>(0, 0), K_r.at<double>(0, 1), K_r.at<double>(0, 2),
                             K_r.at<double>(1, 0), K_r.at<double>(1, 1), K_r.at<double>(1, 2),
                             K_r.at<double>(2, 0), K_r.at<double>(2, 1), K_r.at<double>(2, 2)};
        right_info_msg_.r = {R_r.at<double>(0, 0), R_r.at<double>(0, 1), R_r.at<double>(0, 2),
                             R_r.at<double>(1, 0), R_r.at<double>(1, 1), R_r.at<double>(1, 2),
                             R_r.at<double>(2, 0), R_r.at<double>(2, 1), R_r.at<double>(2, 2)};
        right_info_msg_.p = {P_r.at<double>(0, 0), P_r.at<double>(0, 1), P_r.at<double>(0, 2), P_r.at<double>(0, 3),
                             P_r.at<double>(1, 0), P_r.at<double>(1, 1), P_r.at<double>(1, 2), P_r.at<double>(1, 3),
                             P_r.at<double>(2, 0), P_r.at<double>(2, 1), P_r.at<double>(2, 2), P_r.at<double>(2, 3)};
    }

    void captureAndPublish()
    {
        auto time = this->now();
        captureAndPublishImages(time);
        publishCameraInfo(time);
        // captureAndPublishIMU();
    }

    // void captureAndPublishIMU()
    // {
    //     // Capture and publish IMU data
    //     sl_oc::sensors::data::Imu imu_data = sensor_capture_.getLastIMUData(5000);
    //     auto imu_msg = sensor_msgs::msg::Imu();
    //     imu_msg.header.stamp = this->now();
    //     imu_msg.header.frame_id = "imu_frame";
    //     imu_msg.linear_acceleration.x = imu_data.aX;
    //     imu_msg.linear_acceleration.y = imu_data.aY;
    //     imu_msg.linear_acceleration.z = imu_data.aZ;
    //     imu_msg.angular_velocity.x = imu_data.gX;
    //     imu_msg.angular_velocity.y = imu_data.gY;
    //     imu_msg.angular_velocity.z = imu_data.gZ;
    //     imu_pub_->publish(imu_msg);

    //     RCLCPP_INFO(this->get_logger(), "IMU data: aX=%f, aY=%f, aZ=%f, gX=%f, gY=%f, gZ=%f",
    //                 imu_data.aX, imu_data.aY, imu_data.aZ, imu_data.gX, imu_data.gY, imu_data.gZ);
    // }

    void captureAndPublishImages(rclcpp::Time time)
    {
        cv::Mat frameYUV, frameBGR, left_raw, right_raw;
        const sl_oc::video::Frame frame = video_capture_.getLastFrame();

        RCLCPP_INFO(this->get_logger(), "Raw Camera Frame data: width=%d, height=%d", frame.width, frame.height);

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
            publishImage(left_rectified, left_image_pub_, time);
            publishImage(right_rectified, right_image_pub_, time);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Invalid frame data or dimensions: width=%d, height=%d", frame.width, frame.height);
        }
    }

    void publishImage(const cv::Mat &image, const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub, rclcpp::Time time)
    {
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        img_msg->header.stamp = time;
        img_msg->header.frame_id = "camera_link";
        RCLCPP_INFO(this->get_logger(), "Publishing image: width=%d, height=%d", img_msg->width, img_msg->height);
        pub->publish(*img_msg);
    }

    void publishCameraInfo(rclcpp::Time time)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing camera info");
        left_info_msg_.header.stamp = time;
        right_info_msg_.header.stamp = time;
        left_info_pub_->publish(left_info_msg_);
        right_info_pub_->publish(right_info_msg_);
    }

    // Private members
    sl_oc::video::VideoCapture video_capture_;
    // sl_oc::sensors::SensorCapture sensor_capture_;
    cv::Mat K_l, K_r, D_l, D_r;
    // Output rectification matrices, projection matrices, and Q matrix (for disparity to depth)
    cv::Mat R_l, R_r, P_l, P_r, Q;
    // Rectification maps
    cv::Mat M1l, M2l, M1r, M2r;

    sensor_msgs::msg::CameraInfo left_info_msg_;
    sensor_msgs::msg::CameraInfo right_info_msg_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZedCameraNode>());
    rclcpp::shutdown();
    return 0;
}
