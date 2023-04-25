#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "astra_camera/astra_stereo_u3_interface.hpp"


namespace astra_camera
{

/**
 * @brief Nodelet-wrapper of the BumpBlinkController class
 */
class AstraCameraInterfaceNodelet : public nodelet::Nodelet
{
public:
    AstraCameraInterfaceNodelet(){};
    ~AstraCameraInterfaceNodelet(){}

    /**
    * @brief Initialise the nodelet
    *
    * This function is called, when the nodelet manager loads the nodelet.
    */
    virtual void onInit()
    {   
        ros::NodeHandle nh = this->getNodeHandle();
        ros::NodeHandle pnh = this->getPrivateNodeHandle();

        // resolve node(let) name
        std::string name = nh.getUnresolvedNamespace();

        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        astra_camera_interface_.reset(new AstraStereoU3Interface(nh, pnh));

    }
private:
    boost::shared_ptr<AstraStereoU3Interface> astra_camera_interface_;
};

} // namespace astra_camera

PLUGINLIB_EXPORT_CLASS(astra_camera::AstraCameraInterfaceNodelet, nodelet::Nodelet);