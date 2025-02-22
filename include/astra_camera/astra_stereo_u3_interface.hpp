/**
 * @file astra_stereo_u3_interface.hpp
 * @author Hank Wu (hank880907@gmail.com)
 * @brief A better camera interface for astra stereo u3.
 * @version 0.1
 * @date 2023-04-25
 * 
 * Copyright (c) METALFORM 2022
 * 
 */

#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <openni2/OpenNI.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>
#include "opencv2/core/mat.hpp"
#include "openni2/OniEnums.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <libuvc/libuvc.h>
#include <nodelet/nodelet.h>
// #include <nodelet/nodelet.h>


/**
 * @brief Openni Depth Stream to ROS Image and CameraInfo Interface.
 * 
 */
class OpenniRosInterface: public openni::VideoStream::NewFrameListener
{

public:

    OpenniRosInterface(std::string device_uri, ros::NodeHandle &nh, ros::NodeHandle &pnh, std::string camera_name);

    // constructor for the default device.
    OpenniRosInterface(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~OpenniRosInterface();
    void registerNewFrameCallback(std::function<void(void)> callback)
    {
        new_frame_callbacks_.push_back(callback);
    }

    void onNewFrame(openni::VideoStream& stream) override;
    bool isReady(void);
    std::string getSerialNumber(void);
    void param_cb(void);


private:
    bool is_openni_ready_;
    openni::Device device_;
    std::string serial_number_;
    openni::VideoStream depth_stream_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    std::vector<std::function<void(void)>> new_frame_callbacks_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Publisher image_pub_;

    /* Settings */
    bool align_depth_;
    bool depth_color_sync_;
    std::string camera_name_;

};


/**
 * @brief UvcRosInterface is a class that wraps the uvc library and provides a ros interface.
 * 
 */
class UvcRosInterface
{

public:
    UvcRosInterface(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::string serial_number, std::string camera_name);
    ~UvcRosInterface();
    void onNewFrame(uvc_frame_t *frame);
    void publish(void);
    bool isReady();
    void param_cb();

private:
    bool is_ready_;
    uvc_context_t *ctx_;
    uvc_device_t *dev_;
    uvc_device_handle_t *devh_;
    uvc_stream_ctrl_t ctrl_;
    std::string serial_number_;

    /* ROS */
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Publisher image_pub_;
    ros::Publisher camera_info_pub_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    cv::Mat cv_frame_; 

    /*Settings*/
    int width_=640;
    int height_=480;
    int fps_=30;
    std::string camera_name_;
};



class AstraStereoU3Interface
{

public:
    AstraStereoU3Interface(ros::NodeHandle nh, ros::NodeHandle pnh);
    void param_cb(void);

private:
    std::unique_ptr<UvcRosInterface> uvc_if_;
    std::unique_ptr<OpenniRosInterface> openni_if_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    /*Setting*/
    std::string serial_number_;
    bool enable_uvc_;
    bool enable_openni_;
    std::string camera_name_;
};
