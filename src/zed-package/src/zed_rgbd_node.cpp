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

        // Initialize publishers for ORB SLAM 3
        rgb_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/depth/image_raw", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

        // Initialize publishers for debugging
        disp_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/disp/image_raw", 10);

        // Initialize the camera
        if (!video_capture_.initializeVideo())
        {
            RCLCPP_INFO(this->get_logger(), "Failed to initialize video capture");
            return;
        }

        // Initialize sensor capture
        std::vector<int> devs = sensor_capture_.getDeviceList();
        if (!devs.empty())
        {
            if (!sensor_capture_.initializeSensors(devs[0]))
            {
                RCLCPP_INFO(this->get_logger(), "Failed to initialize sensor capture");
                return;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No devices found for sensor capture");
            return;
        }

        // Create a timer to capture and publish the left camera's image and depth map at 10Hz (has to match value in ZED2_params.yaml)
        img_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ZedCameraNode::captureAndPublishImages, this));
        // Create a timer to capture and publish IMU data at 200Hz (has to match value in ZED2_params.yaml)
        //imu_timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&ZedCameraNode::captureAndPublishIMU, this));
    }

private:
    const int IMAGE_WIDTH = 672;
    const int IMAGE_HEIGHT = 376;

    void initializeCalibrationParams()
    {
        // LEFT and RIGHT camera intrinsic matrices (K) for !!720p resolution!!
        K_l = (cv::Mat_<double>(3, 3) << 263.665, 0, 344.345, 0, 263.5875, 188.1725, 0, 0, 1);
        K_r = (cv::Mat_<double>(3, 3) << 263.665, 0, 333.2625, 0, 263.665, 191.655255, 0, 0, 1);

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
    }

    void captureAndPublishImages()
    {
        std::chrono::high_resolution_clock::time_point start, end, startTotal, endTotal;
        startTotal = std::chrono::high_resolution_clock::now();
        cv::Mat frameYUV, frameBGR, left_raw, right_raw;
        long long duration;

        start = std::chrono::high_resolution_clock::now();
        const sl_oc::video::Frame frame = video_capture_.getLastFrame();
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        // std::cout << "Time taken for image capturing: " << duration << " ms" << std::endl;
        
        // RCLCPP_INFO(this->get_logger(), "Raw Camera Frame data: width=%d, height=%d", frame.width, frame.height);

        if (frame.data != nullptr && frame.width > 0 && frame.height > 0)
        {
            start = std::chrono::high_resolution_clock::now();
            // Convert and split the stereo image
            frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);
            left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
            right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            // std::cout << "Time taken for image conversion and splitting: " << duration << " ms" << std::endl;

            // Rectify the images
            start = std::chrono::high_resolution_clock::now();
            cv::Mat left_rectified, right_rectified;
            cv::remap(left_raw, left_rectified, M1l, M2l, cv::INTER_AREA);
            cv::remap(right_raw, right_rectified, M1r, M2r, cv::INTER_AREA);
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            // std::cout << "Time taken for image rectification: " << duration << " ms" << std::endl;

            // ----> Stereo matching using Semi-Global Block Matching (SGBM), which is more accurate than BM but slower and requires more memory and CPU and GPU power
            start = std::chrono::high_resolution_clock::now();
            cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(0, 16 * 3, 3);
            left_matcher->setMinDisparity(0);
            left_matcher->setNumDisparities(16 * 3);
            left_matcher->setBlockSize(3);
            left_matcher->setP1(8 * 3 * 9);
            left_matcher->setP2(32 * 3 * 9);
            left_matcher->setDisp12MaxDiff(96);
            left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
            left_matcher->setPreFilterCap(31);
            left_matcher->setUniquenessRatio(5);
            left_matcher->setSpeckleWindowSize(31);
            left_matcher->setSpeckleRange(1);
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            // std::cout << "Time taken for SGBM initialization: " << duration << " ms" << std::endl;

            // ----> Compute disparity map
            start = std::chrono::high_resolution_clock::now();
            cv::Mat left_gray, right_gray, left_disp;
            cv::cvtColor(left_rectified, left_gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right_rectified, right_gray, cv::COLOR_BGR2GRAY);
            double resize_factor = 0.5;
            cv::resize(left_gray, left_gray, cv::Size(), resize_factor, resize_factor, cv::INTER_AREA);
            cv::resize(right_gray, right_gray, cv::Size(), resize_factor, resize_factor, cv::INTER_AREA);
            left_matcher->compute(left_gray, right_gray, left_disp);
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            // std::cout << "Time taken for stereo matching: " << duration << " ms" << std::endl;

            // ----> Normalize disparity
            cv::Mat left_disp_float;
            start = std::chrono::high_resolution_clock::now();
            left_disp.convertTo(left_disp_float, CV_32F);
            cv::multiply(left_disp_float, 1.0 / 8.0, left_disp_float); // Combine normalization (disp*1/16) and multiplication by 2 because of the resize
            cv::resize(left_disp_float, left_disp_float, cv::Size(), 1 / resize_factor, 1 / resize_factor, cv::INTER_LINEAR);
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            // std::cout << "Time taken for disparity normalization and inflating: " << duration << " ms" << std::endl;

            double minVal, maxVal;
            cv::minMaxLoc(left_disp_float, &minVal, &maxVal);
            // std::cout << "Disparity map min: " << minVal << " max: " << maxVal << std::endl;

            cv::add(left_disp_float, 1, left_disp_float); // Minimum disparity offset correction

            cv::Mat left_disp_image;
            cv::multiply(left_disp_float, 1. / 96, left_disp_image, 255., CV_8UC1); // Normalization and rescaling
            cv::applyColorMap(left_disp_image, left_disp_image, cv::COLORMAP_JET);  // COLORMAP_INFERNO is better, but it's only available starting from OpenCV v4.1.0

            // ----> Calculate depth map from disparity.
            start = std::chrono::high_resolution_clock::now();
            double fx = 527.33;        // Focal length for the left camera
            double baseline = 120.312; // Baseline in mm
            cv::Mat left_depth_map;
            cv::divide(fx * baseline, left_disp_float, left_depth_map);
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            // std::cout << "Time taken for depth map calculation: " << duration << " ms" << std::endl;

            float central_depth = left_depth_map.at<float>(left_depth_map.rows / 2, left_depth_map.cols / 2);
            // std::cout << "Depth of the central pixel: " << central_depth << " mm" << std::endl;

            cv::minMaxLoc(left_depth_map, &minVal, &maxVal);
            // std::cout << "Depth map min: " << minVal << " max: " << maxVal << std::endl;

            // Publish the rectified images
            start = std::chrono::high_resolution_clock::now();
            auto time = this->now();
            publishImage(left_rectified, rgb_image_pub_, time);
            publishImage(left_disp_image, disp_image_pub_, time);
            publishImage(left_depth_map, depth_image_pub_, time, "32FC1");
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout << "Time taken for image publishing: " << duration << " ms" << std::endl;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Invalid frame data or dimensions: width=%d, height=%d", frame.width, frame.height);
        }
        endTotal = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTotal - startTotal).count();
        std::cout << "Total time taken for function: " << duration << " ms" << std::endl;
    }

    void publishImage(const cv::Mat &image, const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub, rclcpp::Time time, const std::string &encoding = "bgr8")
    {
        // Ensure the encoding matches the image type (BGR for color, 32FC1/16UC1 for depth)
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image).toImageMsg();
        img_msg->header.stamp = time;
        img_msg->header.frame_id = "camera_link";
        // RCLCPP_INFO(this->get_logger(), "Publishing image: width=%d, height=%d, encoding=%s", img_msg->width, img_msg->height, encoding.c_str());
        pub->publish(*img_msg);
    }

    void captureAndPublishIMU()
    {
        // Capture IMU data of last 5000 milliseconds
        sl_oc::sensors::data::Imu imu_data = sensor_capture_.getLastIMUData(5000);
        auto imu_msg = sensor_msgs::msg::Imu();
        auto time = this->now();
        imu_msg.header.stamp = time;
        imu_msg.header.frame_id = "imu_frame";
        imu_msg.linear_acceleration.x = imu_data.aX;
        imu_msg.linear_acceleration.y = imu_data.aY;
        imu_msg.linear_acceleration.z = imu_data.aZ;
        imu_msg.angular_velocity.x = imu_data.gX;
        imu_msg.angular_velocity.y = imu_data.gY;
        imu_msg.angular_velocity.z = imu_data.gZ;

        // publish IMU data
        RCLCPP_INFO(this->get_logger(), "IMU data: aX=%f, aY=%f, aZ=%f, gX=%f, gY=%f, gZ=%f",
                    imu_data.aX, imu_data.aY, imu_data.aZ, imu_data.gX, imu_data.gY, imu_data.gZ);
        imu_pub_->publish(imu_msg);
    }

    // Private members
    sl_oc::video::VideoCapture video_capture_;
    sl_oc::sensors::SensorCapture sensor_capture_;
    cv::Mat K_l, K_r, D_l, D_r;
    // Output rectification matrices, projection matrices, and Q matrix (for disparity to depth)
    cv::Mat R_l, R_r, P_l, P_r, Q;
    // Rectification maps
    cv::Mat M1l, M2l, M1r, M2r;

    // Publishers for ORB SLAM 3
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // Publisher for debugging
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disp_image_pub_;

    rclcpp::TimerBase::SharedPtr img_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::VGA;
    rclcpp::spin(std::make_shared<ZedCameraNode>(params));
    rclcpp::shutdown();
    return 0;
}
