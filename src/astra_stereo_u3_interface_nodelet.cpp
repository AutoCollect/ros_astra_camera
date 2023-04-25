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

    OpenniRosInterface(std::string device_uri, ros::NodeHandle &nh, ros::NodeHandle &pnh):
        is_openni_ready_(true),
        device_uri_(device_uri),
        nh_(nh),
        pnh_(pnh),
        image_transport_(nh_)
    {
        param_cb();

        openni::Status status = openni::OpenNI::initialize();
        if (status != openni::STATUS_OK) 
        {
            ROS_ERROR("Failed to initialize OpenNI: %s", openni::OpenNI::getExtendedError());
            is_openni_ready_ = false;
            return;
        }
        
        // if no device uri is given, use the first available device
        if (device_uri_.empty())
        {
            ROS_WARN("No device uri given, using the first available device");
            openni::Array<openni::DeviceInfo> devices;
            openni::OpenNI::enumerateDevices(&devices);

            if (devices.getSize() == 0) 
            {
                ROS_ERROR("No device connected");
                is_openni_ready_ = false;
                return;
            }

            std::cout << "Found " << devices.getSize() << " devices:" << std::endl;
            for (int i = 0; i < devices.getSize(); ++i) 
            {
                std::cout << " uri: " << devices[i].getUri() << std::endl;
                std::cout << " name: " << devices[i].getName() << std::endl;
                std::cout << " vendor " << devices[i].getVendor() << std::endl;
                std::cout << " product id: " << devices[i].getUsbProductId() << std::endl;
                std::cout << " vendor id: " << devices[i].getUsbVendorId() << std::endl;
            }

            device_uri_ = std::string(devices[0].getUri());
            ROS_INFO("Using device uri: %s", device_uri_.c_str());
        }

        // try to open the uri
        status = device_.open(device_uri_.c_str());
        if (status != openni::STATUS_OK) 
        {
            ROS_ERROR("Failed to open device: %s", openni::OpenNI::getExtendedError());
            is_openni_ready_ = false;
            return;
        }

        // get the serial number.
        char serial_number[64];
        int data_size = sizeof(serial_number);
        device_.getProperty(openni::OBEXTENSION_ID_SERIALNUMBER, serial_number, &data_size);
        serial_number_ = std::string(serial_number);
        ROS_INFO("serial number: %s", serial_number_.c_str());

        // disable mirror. (does not seem to have effect.)
        depth_stream_.setMirroringEnabled(false);

        // open depth stream.
        status = depth_stream_.create(device_, openni::SENSOR_DEPTH);
        if (status != openni::STATUS_OK)
        {
            ROS_ERROR("Failed to create depth stream: %s", openni::OpenNI::getExtendedError());
            is_openni_ready_ = false;
            return;
        }

        // depth color sync
        if (depth_color_sync_)
        {
            status = device_.setDepthColorSyncEnabled(true);
            if (status != openni::STATUS_OK)
            {
                ROS_WARN("Failed to enable depth color sync: %s", openni::OpenNI::getExtendedError());
            }
        }

        // set depth alignment
        if (align_depth_)
        {
            if (!device_.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
                ROS_WARN_STREAM("Current do not support IMAGE_REGISTRATION_DEPTH_TO_COLOR");
            }
            status = device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
            if (status != openni::STATUS_OK)
            {
                ROS_WARN("Failed to set image registration mode: %s", openni::OpenNI::getExtendedError());
            }
        }
            
        
        // initialize image transport
        image_pub_ = image_transport_.advertise("depth", 10);

        // attach callback
        status = depth_stream_.addNewFrameListener(this);

        // start the stream
        status = depth_stream_.start();
        if (status != openni::STATUS_OK)
        {
            ROS_ERROR("Failed to start depth stream: %s", openni::OpenNI::getExtendedError());
            is_openni_ready_ = false;
            return;
        }
    }

    // constructor for the default device.
    OpenniRosInterface(ros::NodeHandle &nh, ros::NodeHandle &pnh):
        OpenniRosInterface("", nh, pnh)
    {

    }

    ~OpenniRosInterface()
    {
        depth_stream_.destroy();
        device_.close();
        openni::OpenNI::shutdown();
    }

    void onNewFrame(openni::VideoStream& stream) override
    {
        openni::VideoFrameRef frame;
        auto status = stream.readFrame(&frame);
        if (status != openni::STATUS_OK) {
            ROS_ERROR("Failed to read frame: %s", openni::OpenNI::getExtendedError());
            return;
        }

        if (!frame.isValid())
        {
            ROS_ERROR("Recived an invalid frame");
            return;
        }

        if (frame.getSensorType() != openni::SENSOR_DEPTH)
        {
            ROS_ERROR("Recived a frame from a non depth sensor");
            return;
        }

        if (frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM)
        {
            ROS_ERROR("Recived a frame with an invalid pixel format %d", frame.getVideoMode().getPixelFormat());
            return;
        }

        // convert to opencv
        // the pixel will be 16 bit unsigned int
        cv::Mat cv_image;

        if (frame.getStrideInBytes() != frame.getWidth() * sizeof(uint16_t))
        {
            // if the stride is not equal to the width, it means that the data is not continuous
            std::vector<uint16_t> image;
            image.reserve(frame.getWidth() * frame.getHeight());
            for(int i = 0; i < frame.getHeight(); ++i)
            {   
                auto dest_start = image.data() + i * frame.getWidth();
                auto src_start = (uint16_t*) frame.getData() + i * (frame.getStrideInBytes()/2);
                memcpy(dest_start, src_start, frame.getWidth() * sizeof(uint16_t));
            }
            cv_image = cv::Mat(frame.getHeight(), frame.getWidth(), CV_16UC1, image.data());

        }else {
            // otherwise, just cast the data to a cv mat.
            cv_image = cv::Mat(frame.getHeight(), frame.getWidth(), CV_16UC1, (void*)frame.getData());
        }
        cv::flip(cv_image, cv_image, 1);
        // convert to ros image
        sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(std_msgs::Header(), "mono16", cv_image).toImageMsg();
        // publish
        image_pub_.publish(ros_image);
    }

    bool isReady(void)
    {
        return is_openni_ready_;
    }

    std::string getSerialNumber(void)
    {
        return serial_number_;
    }

    void param_cb(void)
    {
        if (!pnh_.param<bool>("align_depth", align_depth_, true))
        {
            ROS_WARN("Failed to get align_depth parameter, using default value: %d", align_depth_);
        }
        if (!pnh_.param<bool>("depth_color_sync", depth_color_sync_, true))
        {
            ROS_WARN("Failed to get depth_color_sync parameter, using default value: %d", depth_color_sync_);
        }
    }


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
    UvcRosInterface(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::string serial_number):
        is_ready_(true),
        serial_number_(serial_number),
        nh_(nh),
        pnh_(pnh),
        image_transport_(nh_)

    {
        param_cb();
        uvc_error_t res;
        res = uvc_init(&ctx_, NULL);
        if (res < 0) {
            ROS_ERROR("uvc_init error");
            is_ready_ = false;
            return;
        }

        // if no serial number is provided, use the first device found.
        if (serial_number_.empty())
        {
            ROS_WARN("No serial number provided, using the first device found");
            res = uvc_find_device(ctx_, &dev_, 0, 0, NULL);
        }
        else 
        {
            res = uvc_find_device(ctx_, &dev_, 0, 0, serial_number_.c_str());
        }

        if (res < 0) 
        {
            ROS_ERROR("uvc_find_device error");
            is_ready_ = false;
            return;
        }

        // open the device
        res = uvc_open(dev_, &devh_);
        if (res < 0) 
        {
            ROS_ERROR("uvc_open error");
            is_ready_ = false;
            return;
        }

        // set the control
        res = uvc_get_stream_ctrl_format_size(
            devh_, &ctrl_, UVC_FRAME_FORMAT_MJPEG,
            width_, height_, fps_
        );

        if (res < 0) 
        {
            ROS_ERROR("Failed to get stream control");
            is_ready_ = false;
            return;
        }

        // set up image publisher
        image_pub_ = image_transport_.advertise("rgb", 10);

        // register callback.
        res = uvc_start_streaming(devh_, 
                                &ctrl_, 
                                [](uvc_frame_t* frame, void* user_ptr) {
                                    UvcRosInterface* uvc_ros_interface = static_cast<UvcRosInterface*>(user_ptr);
                                    uvc_ros_interface->onNewFrame(frame);
                                },
                                this,
                                0);
        if (res != UVC_SUCCESS) 
        {
            ROS_ERROR("Failed to start streaming");
            is_ready_ = false;
            return;
        }

    }

    ~UvcRosInterface()
    {
        uvc_stop_streaming(devh_);
        uvc_close(devh_);
        uvc_unref_device(dev_);
        uvc_exit(ctx_);
    }

    void onNewFrame(uvc_frame_t *frame)
    {
        // this appears to be the only format supported by astra stereo u3.
        if (frame->frame_format != UVC_FRAME_FORMAT_MJPEG) 
        {
            ROS_ERROR("Recived a frame with an invalid pixel format %d. Expected MJPEG.", frame->frame_format);
            return;
        }
        std::vector<uchar> mjpeg_data((uchar*)frame->data, (uchar*) frame->data + frame->data_bytes);
        cv::Mat cv_frame = cv::imdecode(mjpeg_data, cv::IMREAD_COLOR);
        sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_frame).toImageMsg();
        image_pub_.publish(ros_image);
    }

    bool isReady()
    {
        return is_ready_;
    }

    void param_cb()
    {
        if (!pnh_.param<int>("width", width_, 640))
        {
            // ROS_WARN("Failed to get width parameter, using default value: %d", width_);
        }

        if (!pnh_.param<int>("height", height_, 480))
        {
            // ROS_WARN("Failed to get height parameter, using default value: %d", height_);
        }

        if (!pnh_.param<int>("fps", fps_, 30))
        {
            // ROS_WARN("Failed to get fps parameter, using default value: %d", fps_);
        }
    }

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
    AstraStereoU3Interface(ros::NodeHandle nh, ros::NodeHandle pnh):
        nh_(nh),
        pnh_(pnh)
    {
        param_cb();

        auto openni_if_pnh = ros::NodeHandle(pnh_.getNamespace() + "/openni");
        openni_if_ = std::make_unique<OpenniRosInterface>(device_uri_, nh_, openni_if_pnh);
        if (!openni_if_->isReady())
        {
            ROS_ERROR("Failed to initialize openni");
            return;
        }

        auto uvc_if_pnh = ros::NodeHandle(pnh_.getNamespace() + "/uvc");
        uvc_if_ = std::make_unique<UvcRosInterface>(nh_, uvc_if_pnh, openni_if_->getSerialNumber());
        if (!uvc_if_->isReady())
        {
            ROS_ERROR("Failed to initialize uvc");
            return;
        }

    }

    void param_cb(void)
    {
        if (!pnh_.param<std::string>("device_uri", device_uri_, ""))
        {
            ROS_WARN("URI is not specified. Using the first device found.");
        }
        else {
            ROS_INFO("Using device with URI: %s", device_uri_.c_str());
        }
        if (!pnh_.param<bool>("enable_uvc", enable_uvc_, true))
        {
            ROS_WARN("Failed to get enable_uvc parameter, using default value: %d", enable_uvc_);
        }
        if (!pnh_.param<bool>("enable_openni", enable_openni_, true))
        {
            ROS_WARN("Failed to get enable_openni parameter, using default value: %d", enable_openni_);
        }
    }

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



int main(int argc, char** argv)
{
    ros::init(argc, argv, "openni_ros_interface");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    AstraStereoU3Interface astra_if(nh, pnh);
    ros::spin();
    return 0;
}

