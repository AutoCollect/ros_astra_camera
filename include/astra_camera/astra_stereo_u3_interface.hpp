#include <cstdint>
#include <cstring>
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

    OpenniRosInterface(std::string device_uri, ros::NodeHandle &nh, ros::NodeHandle &pnh);

    // constructor for the default device.
    OpenniRosInterface(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~OpenniRosInterface();
    void onNewFrame(openni::VideoStream& stream) override;
    bool isReady(void);
    std::string getSerialNumber(void);
    void param_cb(void);


private:
    bool is_openni_ready_;
    std::string device_uri_;
    openni::Device device_;
    std::string serial_number_;
    openni::VideoStream depth_stream_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Publisher image_pub_;

    /* Settings */
    bool align_depth_;
    bool depth_color_sync_;

};


/**
 * @brief UvcRosInterface is a class that wraps the uvc library and provides a ros interface.
 * 
 */
class UvcRosInterface
{

public:
    UvcRosInterface(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::string serial_number);
    ~UvcRosInterface();
    void onNewFrame(uvc_frame_t *frame);
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

    /*Settings*/
    int width_=640;
    int height_=480;
    int fps_=30;
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
    std::string device_uri_;
    bool enable_uvc_;
    bool enable_openni_;
};
