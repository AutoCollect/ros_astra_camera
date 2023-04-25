#include "astra_camera/astra_stereo_u3_interface.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "openni_ros_interface");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    AstraStereoU3Interface astra_if(nh, pnh);
    ros::spin();
    return 0;
}
