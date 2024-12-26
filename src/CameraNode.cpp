#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/stream.h>
#include <libcamera/control_ids.h>
#include <libcamera/request.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class CameraNode
{
public:
    CameraNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
        : nh_(nh),
          nh_private_(nh_private),
          camera_info_manager_(nh, "camera")
    {
        // Publishers
        pub_image_ = nh_.advertise<sensor_msgs::Image>("image_raw", 1);
        pub_image_compressed_ = nh_.advertise<sensor_msgs::CompressedImage>("image_raw/compressed", 1);
        pub_camera_info_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

        // Initialize camera manager
        camera_manager_ = std::make_shared<libcamera::CameraManager>();
        camera_manager_->start();

        // Select camera
        auto cameras = camera_manager_->cameras();
        if (cameras.empty())
        {
            ROS_ERROR("No cameras available");
            ros::shutdown();
            return;
        }
        camera_ = cameras.front();
        if (camera_->acquire())
        {
            ROS_ERROR("Failed to acquire camera");
            ros::shutdown();
            return;
        }

        // Configure camera
        configureCamera();

        // Start capturing
        startCapture();
    }

    ~CameraNode()
    {
        if (camera_)
        {
            camera_->stop();
            camera_->release();
        }
        if (camera_manager_)
        {
            camera_manager_->stop();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher pub_image_;
    ros::Publisher pub_image_compressed_;
    ros::Publisher pub_camera_info_;
    camera_info_manager::CameraInfoManager camera_info_manager_;

    std::shared_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    libcamera::Stream *stream_;

    void configureCamera()
    {
        auto config = camera_->generateConfiguration({libcamera::StreamRole::VideoRecording});
        if (!config)
        {
            ROS_ERROR("Failed to generate camera configuration");
            ros::shutdown();
            return;
        }

        auto &stream_config = config->at(0);
        stream_config.size.width = 640;
        stream_config.size.height = 480;
        stream_config.pixelFormat = libcamera::formats::RGB888;

        if (camera_->configure(config.get()) < 0)
        {
            ROS_ERROR("Failed to configure camera");
            ros::shutdown();
            return;
        }

        stream_ = stream_config.stream();
    }

    void startCapture()
    {
        if (camera_->start())
        {
            ROS_ERROR("Failed to start camera");
            ros::shutdown();
            return;
        }

        ros::Rate loop_rate(30);
        while (ros::ok())
        {
            captureFrame();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void captureFrame()
    {
        auto request = camera_->createRequest();
        if (!request)
        {
            ROS_ERROR("Failed to create request");
            return;
        }

        camera_->queueRequest(request.get());

        const auto &buffers = request->buffers();
        for (const auto &pair : buffers)
        {
            auto buffer = pair.second;
            auto data = buffer->planes()[0].fd.get();
            cv::Mat frame(cv::Size(640, 480), CV_8UC3, data);

            publishFrame(frame);
        }
    }

    void publishFrame(const cv::Mat &frame)
    {
        auto msg_image = boost::make_shared<sensor_msgs::Image>();
        auto msg_image_compressed = boost::make_shared<sensor_msgs::CompressedImage>();
        auto msg_camera_info = boost::make_shared<sensor_msgs::CameraInfo>();

        // Fill Image message
        msg_image->header.stamp = ros::Time::now();
        msg_image->header.frame_id = "camera";
        msg_image->height = frame.rows;
        msg_image->width = frame.cols;
        msg_image->encoding = sensor_msgs::image_encodings::RGB8;
        msg_image->step = frame.cols * frame.elemSize();
        msg_image->data.assign(frame.data, frame.data + frame.total() * frame.elemSize());

        // Fill CompressedImage message
        msg_image_compressed->header = msg_image->header;
        msg_image_compressed->format = "jpeg";
        cv::imencode(".jpg", frame, msg_image_compressed->data);

        // Fill CameraInfo message
        *msg_camera_info = camera_info_manager_.getCameraInfo();
        msg_camera_info->header = msg_image->header;

        pub_image_.publish(msg_image);
        pub_image_compressed_.publish(msg_image_compressed);
        pub_camera_info_.publish(msg_camera_info);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    CameraNode camera_node(nh, nh_private);

    return 0;
}
