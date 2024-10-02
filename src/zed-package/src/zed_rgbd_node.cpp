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

        // Initialize publishers adjusted to ORB SLAM 3
        rgb_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/depth/image_raw", 10);
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

        // Create a timer to capture and publish data at 2Hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ZedCameraNode::captureAndPublish, this));
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

        cv::Mat T = (cv::Mat_<double>(3, 1) << -0.120312, 0.000018, -0.0007697); // Translation vector in meters (Tx, Ty, Tz)

        // Image size (width and height)
        cv::Size imageSize(IMAGE_WIDTH, IMAGE_HEIGHT);

        // Compute rectification transforms
        cv::stereoRectify(K_l, D_l, K_r, D_r, imageSize, R, T, R_l, R_r, P_l, P_r, Q);

        // Create rectification maps for left and right images
        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l, imageSize, CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r, imageSize, CV_32F, M1r, M2r);
    }

    void captureAndPublish()
    {
        auto time = this->now();
        captureAndPublishImages(time);
    }

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

            cv::Mat left_resized, right_resized;
            cv::resize(left_raw, left_resized, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));
            cv::resize(right_raw, right_resized, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));

            // Rectify the images
            cv::Mat left_rectified, right_rectified;
            cv::remap(left_resized, left_rectified, M1l, M2l, cv::INTER_AREA);
            cv::remap(right_resized, right_rectified, M1r, M2r, cv::INTER_AREA);

            // ----> Stereo matching using Semi-Global Block Matching (SGBM)
            cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(0, 16, 3);
            left_matcher->setMinDisparity(0);
            left_matcher->setNumDisparities(16);
            left_matcher->setBlockSize(3);
            left_matcher->setP1(8 * 3 * 9);
            left_matcher->setP2(32 * 3 * 9);
            left_matcher->setDisp12MaxDiff(1);
            left_matcher->setMode(cv::StereoSGBM::MODE_SGBM);
            left_matcher->setPreFilterCap(63);
            left_matcher->setUniquenessRatio(10);
            left_matcher->setSpeckleWindowSize(100);
            left_matcher->setSpeckleRange(32);

            // ----> Compute disparity map
            cv::Mat left_disp, left_disp_float;
            left_matcher->compute(left_rectified, right_rectified, left_disp);

            // ----> Normalize disparity
            left_disp.convertTo(left_disp_float, CV_32FC1);
            cv::multiply(left_disp_float, 1.0 / 16.0, left_disp_float); // Scale disparity

            // ----> Calculate depth map from disparity.
            cv::Mat left_depth_map;
            double num = static_cast<double>(1054.66 * -0.120312);
            cv::divide(num, left_disp_float, left_depth_map);
            //cv::reprojectImageTo3D(left_disp_float, left_depth_map, Q, true, CV_32F);

            // Publish the rectified images
            publishImage(left_rectified, rgb_image_pub_, time);
            publishImage(right_rectified, rgb_image_pub_, time);
            publishImage(left_depth_map, depth_image_pub_, time, "32FC1");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Invalid frame data or dimensions: width=%d, height=%d", frame.width, frame.height);
        }
    }

    void publishImage(const cv::Mat &image, const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub, rclcpp::Time time, const std::string &encoding = "bgr8")
    {
        // Ensure the encoding matches the image type (BGR for color, 32FC1/16UC1 for depth)
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image).toImageMsg();
        img_msg->header.stamp = time;
        img_msg->header.frame_id = "camera_link";
        RCLCPP_INFO(this->get_logger(), "Publishing image: width=%d, height=%d, encoding=%s", img_msg->width, img_msg->height, encoding.c_str());
        pub->publish(*img_msg);
    }

    // Private members
    sl_oc::video::VideoCapture video_capture_;
    // sl_oc::sensors::SensorCapture sensor_capture_;
    cv::Mat K_l, K_r, D_l, D_r;
    // Output rectification matrices, projection matrices, and Q matrix (for disparity to depth)
    cv::Mat R_l, R_r, P_l, P_r, Q;
    // Rectification maps
    cv::Mat M1l, M2l, M1r, M2r;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
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
