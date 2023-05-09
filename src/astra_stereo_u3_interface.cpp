/**
 * @file astra_stereo_u3_interface.cpp
 * @author Hank Wu (hank880907@gmail.com)
 * @brief A better camera interface.
 * @version 0.1
 * @date 2023-04-25
 * 
 * Copyright (c) METALFORM 2022
 * 
 */

#include "astra_camera/astra_stereo_u3_interface.hpp"
#include "sensor_msgs/CameraInfo.h"


OpenniRosInterface::OpenniRosInterface(std::string serial_number, ros::NodeHandle &nh, ros::NodeHandle &pnh, std::string camera_name):
    is_openni_ready_(true),
    nh_(nh),
    pnh_(pnh),
    serial_number_(serial_number),
    image_transport_(nh_),
    camera_name_(camera_name)
{
    param_cb();

    openni::Status status = openni::OpenNI::initialize();
    if (status != openni::STATUS_OK) 
    {
        ROS_ERROR("Failed to initialize OpenNI: %s", openni::OpenNI::getExtendedError());
        is_openni_ready_ = false;
        return;
    }
    
    openni::Array<openni::DeviceInfo> devices;
    openni::OpenNI::enumerateDevices(&devices);

    if (devices.getSize() == 0) 
    {
        ROS_ERROR("No device connected");
        is_openni_ready_ = false;
        return;
    }

    std::map<std::string, std::string> device_map; // map from serial number to uri

    std::cout << "Found " << devices.getSize() << " devices:" << std::endl;
    for (int i = 0; i < devices.getSize(); ++i) 
    {
        std::cout << " uri: " << devices[i].getUri() << std::endl;
        std::cout << " name: " << devices[i].getName() << std::endl;
        std::cout << " vendor " << devices[i].getVendor() << std::endl;
        std::cout << " product id: " << devices[i].getUsbProductId() << std::endl;
        std::cout << " vendor id: " << devices[i].getUsbVendorId() << std::endl;
        openni::Device device;
        status = device.open(devices[i].getUri());
        if (status == openni::STATUS_OK) 
        {
            char serial_number_char[64];
            int data_size = sizeof(serial_number_char);
            device.getProperty(openni::OBEXTENSION_ID_SERIALNUMBER, serial_number_char, &data_size);
            auto serial_number = std::string(serial_number_char);
            device_map[serial_number] = devices[i].getUri(); 
        }
        device.close();
    }

    if (serial_number_ == "")
    {
        serial_number_ = device_map.begin()->first;
        ROS_INFO("No serial number specified. Using the first device with serial number: %s", serial_number_.c_str());
    }

    if (device_map.find(serial_number_) == device_map.end())
    {
        ROS_ERROR("Cannot find device with serial number: %s", serial_number_.c_str());
        is_openni_ready_ = false;
        return;
    }

    std::string device_uri = device_map[serial_number_];
    
    ROS_INFO("Using device uri: %s", device_uri.c_str());


    // try to open the uri
    status = device_.open(device_uri.c_str());
    if (status != openni::STATUS_OK) 
    {
        ROS_ERROR("Failed to open device: %s", openni::OpenNI::getExtendedError());
        is_openni_ready_ = false;
        return;
    }

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
OpenniRosInterface::OpenniRosInterface(ros::NodeHandle &nh, ros::NodeHandle &pnh):
    OpenniRosInterface("", nh, pnh, "astra")
{

}

OpenniRosInterface::~OpenniRosInterface()
{
    depth_stream_.destroy();
    device_.close();
    openni::OpenNI::shutdown();
}

void OpenniRosInterface::onNewFrame(openni::VideoStream& stream)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = camera_name_ + "_optical_frame";

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

    sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(header, "mono16", cv_image).toImageMsg();
    // publish
    image_pub_.publish(ros_image);

    for (auto &cb : new_frame_callbacks_)
    {
        cb();
    }
}

bool OpenniRosInterface::isReady(void)
{
    return is_openni_ready_;
}

std::string OpenniRosInterface::getSerialNumber(void)
{
    return serial_number_;
}

void OpenniRosInterface::param_cb(void)
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




UvcRosInterface::UvcRosInterface(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::string serial_number, std::string camera_name):
    is_ready_(true),
    serial_number_(serial_number),
    nh_(nh),
    pnh_(pnh),
    image_transport_(nh_),
    camera_name_(camera_name)

{
    param_cb();
    uvc_error_t res;
    res = uvc_init(&ctx_, NULL);
    if (res < 0) {
        ROS_ERROR("uvc_init error");
        is_ready_ = false;
        return;
    }

    camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_));
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
    camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

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

UvcRosInterface::~UvcRosInterface()
{
    uvc_stop_streaming(devh_);
    uvc_close(devh_);
    uvc_unref_device(dev_);
    uvc_exit(ctx_);
}

void UvcRosInterface::onNewFrame(uvc_frame_t *frame)
{


    // this appears to be the only format supported by astra stereo u3.
    if (frame->frame_format != UVC_FRAME_FORMAT_MJPEG) 
    {
        ROS_ERROR("Recived a frame with an invalid pixel format %d. Expected MJPEG.", frame->frame_format);
        return;
    }

    


    std::vector<uchar> mjpeg_data((uchar*)frame->data, (uchar*) frame->data + frame->data_bytes);
    cv_frame_ = cv::imdecode(mjpeg_data, cv::IMREAD_COLOR);

}


void UvcRosInterface::publish(void)
{
    auto now = ros::Time::now();
    sensor_msgs::CameraInfoPtr camera_info(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
    camera_info->header.frame_id = camera_name_ + "_optical_frame";
    camera_info->header.stamp = now;
    std_msgs::Header header;
    header.stamp = camera_info->header.stamp;
    header.frame_id = camera_info->header.frame_id;
    sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(header, "bgr8", cv_frame_).toImageMsg();
    image_pub_.publish(ros_image);
    camera_info_pub_.publish(camera_info);
}


bool UvcRosInterface::isReady()
{
    return is_ready_;
}

void UvcRosInterface::param_cb()
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



AstraStereoU3Interface::AstraStereoU3Interface(ros::NodeHandle nh, ros::NodeHandle pnh):
    nh_(nh),
    pnh_(pnh)
{
    param_cb();

    auto openni_if_pnh = ros::NodeHandle(pnh_.getNamespace() + "/openni");
    openni_if_ = std::make_unique<OpenniRosInterface>(serial_number_, nh_, openni_if_pnh, camera_name_);
    if (!openni_if_->isReady())
    {
        ROS_ERROR("Failed to initialize openni");
        return;
    }

    auto uvc_if_pnh = ros::NodeHandle(pnh_.getNamespace() + "/uvc");
    uvc_if_ = std::make_unique<UvcRosInterface>(nh_, uvc_if_pnh, openni_if_->getSerialNumber(), camera_name_);
    if (!uvc_if_->isReady())
    {
        ROS_ERROR("Failed to initialize uvc");
        return;
    }

    openni_if_->registerNewFrameCallback(boost::bind(&UvcRosInterface::publish, uvc_if_.get()));
}

void AstraStereoU3Interface::param_cb(void)
{
    if (!pnh_.param<std::string>("serial_number", serial_number_, ""))
    {
        ROS_WARN("serial_number is not specified. Using the first device found.");
    }
    else {
        ROS_INFO("Using device with URI: %s", serial_number_.c_str());
    }
    if (!pnh_.param<bool>("enable_uvc", enable_uvc_, true))
    {
        ROS_WARN("Failed to get enable_uvc parameter, using default value: %d", enable_uvc_);
    }
    if (!pnh_.param<bool>("enable_openni", enable_openni_, true))
    {
        ROS_WARN("Failed to get enable_openni parameter, using default value: %d", enable_openni_);
    }

    if (!pnh_.param("camera_name", camera_name_, std::string("astra")))
    {
        ROS_WARN("Failed to get camera_name parameter, using default value: %s", camera_name_.c_str());
    }

    
}
