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
    ZedCameraNode(sl_oc::video::VideoParams params)
        : Node("zed_camera_node"),
          video_capture_(params)
    {
        // Initialize camera calibration parameters
        initializeCalibrationParams();

        // Initialize publishers adjusted to ORB SLAM 3
        rgb_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        rgb_image_pub_right = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw/right", 10);
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
        // LEFT and RIGHT camera intrinsic matrices (K) for !!720p resolution!!
        K_l = (cv::Mat_<double>(3, 3) << 527.33, 0, 657.69, 0, 527.175, 361.345, 0, 0, 1);
        K_r = (cv::Mat_<double>(3, 3) << 527.365, 0, 635.525, 0, 527.15, 368.3105, 0, 0, 1);

        // LEFT and RIGHT distortion coefficients
        D_l = (cv::Mat_<double>(1, 5) << -0.0406, 0.0095, -0.0006, -0.0001, -0.0047);
        D_r = (cv::Mat_<double>(1, 5) << -0.0414, 0.0094, 0.0001, 0.0002, -0.0046);

        // Rotation and translation between the cameras
        cv::Mat R;
        cv::Mat rvec = (cv::Mat_<double>(3, 1) << -0.0011, 0.0000, -0.0001); // Rotation vector (Rx, Ry, Rz in radians)
        cv::Rodrigues(rvec, R);                                              // Convert the rotation vector into a 3x3 rotation matrix

        cv::Mat T = (cv::Mat_<double>(3, 1) << 120.312, 0.018, -0.7697); // Translation vector from right camera to left camera in millimeters (Tx, Ty, Tz)

        // Image size (width and height)
        cv::Size imageSize(IMAGE_WIDTH, IMAGE_HEIGHT);

        // Compute rectification transforms
        std::cout << "Rotation matrix: " << R << std::endl;
        cv::stereoRectify(K_l, D_l, K_r, D_r, imageSize, R, T, R_l, R_r, P_l, P_r, Q, cv::CALIB_ZERO_DISPARITY, 0);

        std::cout << "Q matrix: " << Q << std::endl;

        // Create rectification maps for left and right images
        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l, imageSize, CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r, imageSize, CV_32F, M1r, M2r);

        std::cout << "Map M1l (0,0): " << M1l.at<float>(0, 0) << std::endl;
        std::cout << "Map M1l (2,3): " << M1l.at<float>(2, 3) << std::endl;
        std::cout << "Map M1l (5,5): " << M1l.at<float>(5, 5) << std::endl;

        std::cout << "Map M2l (0,0): " << M2l.at<float>(0, 0) << std::endl;
        std::cout << "Map M2l (2,3): " << M2l.at<float>(2, 3) << std::endl;
        std::cout << "Map M2l (5,5): " << M2l.at<float>(5, 5) << std::endl;

        std::cout << "Map M1r (0,0): " << M1r.at<float>(0, 0) << std::endl;
        std::cout << "Map M1r (2,3): " << M1r.at<float>(2, 3) << std::endl;
        std::cout << "Map M1r (5,5): " << M1r.at<float>(5, 5) << std::endl;

        std::cout << "Map M2r (0,0): " << M2r.at<float>(0, 0) << std::endl;
        std::cout << "Map M2r (2,3): " << M2r.at<float>(2, 3) << std::endl;
        std::cout << "Map M2r (5,5): " << M2r.at<float>(5, 5) << std::endl;
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

            // Rectify the images
            cv::Mat left_rectified, right_rectified;
            cv::remap(left_raw, left_rectified, M1l, M2l, cv::INTER_AREA);
            cv::remap(right_raw, right_rectified, M1r, M2r, cv::INTER_AREA);

            // ----> Stereo matching using Semi-Global Block Matching (SGBM), which is more accurate than BM but slower and requires more memory and CPU and GPU power
            cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(0, 16 * 6, 3);
            left_matcher->setMinDisparity(0);
            left_matcher->setNumDisparities(16 * 6);
            left_matcher->setBlockSize(3);
            left_matcher->setP1(8 * 3 * 9);
            left_matcher->setP2(32 * 3 * 9);
            left_matcher->setDisp12MaxDiff(96);
            left_matcher->setMode(cv::StereoSGBM::MODE_SGBM);
            left_matcher->setPreFilterCap(63);
            left_matcher->setUniquenessRatio(5);
            left_matcher->setSpeckleWindowSize(255);
            left_matcher->setSpeckleRange(1);

            // ----> Compute disparity map
            cv::Mat left_gray, right_gray, left_disp;
            cv::cvtColor(left_rectified, left_gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right_rectified, right_gray, cv::COLOR_BGR2GRAY);
            left_matcher->compute(left_gray, right_gray, left_disp);

            // ----> Normalize disparity
            cv::Mat left_disp_float;
            left_disp.convertTo(left_disp_float, CV_32F);
            cv::multiply(left_disp_float, 1.0 / 16.0, left_disp_float); // Divide by 16 to get the disparity in pixels

            double minVal, maxVal;
            cv::minMaxLoc(left_disp_float, &minVal, &maxVal);
            std::cout << "Disparity map min: " << minVal << " max: " << maxVal << std::endl;

            cv::add(left_disp_float, 1, left_disp_float);  // Minimum disparity offset correction

            cv::Mat left_disp_image;
            cv::multiply(left_disp_float, 1. / 96, left_disp_image, 255., CV_8UC1); // Normalization and rescaling
            cv::applyColorMap(left_disp_image, left_disp_image, cv::COLORMAP_INFERNO); // COLORMAP_INFERNO is better, but it's only available starting from OpenCV v4.1.0

            // ----> Calculate depth map from disparity.
            double fx = 527.33;        // Focal length for the left camera
            double baseline = 120.312; // Baseline in mm
            cv::Mat left_depth_map;
            cv::divide(fx * baseline, left_disp_float, left_depth_map);

            // Calculate depth map using the Q matrix
            // cv::Mat left_depth_map;
            // cv::reprojectImageTo3D(left_disp_float, left_depth_map, Q, true);

            // // Extract the Z channel from the 3D points (depth information)
            // std::vector<cv::Mat> channels(3);
            // cv::split(left_depth_map, channels);
            // left_depth_map = channels[2];

            float central_depth = left_depth_map.at<float>(left_depth_map.rows / 2, left_depth_map.cols / 2);
            std::cout << "Depth of the central pixel: " << central_depth << " mm" << std::endl;

            cv::minMaxLoc(left_depth_map, &minVal, &maxVal);
            std::cout << "Depth map min: " << minVal << " max: " << maxVal << std::endl;

            // Publish the rectified images
            publishImage(left_rectified, rgb_image_pub_, time);
            publishImage(left_disp_image, rgb_image_pub_right, time);
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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_image_pub_right;
    // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD720;
    rclcpp::spin(std::make_shared<ZedCameraNode>(params));
    rclcpp::shutdown();
    return 0;
}
