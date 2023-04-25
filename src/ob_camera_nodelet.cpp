#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "astra_camera/ob_camera_node_factory.h"


namespace astra_camera
{

/**
 * @brief Nodelet-wrapper of the BumpBlinkController class
 */
class ObCameraNodelet : public nodelet::Nodelet
{
public:
    ObCameraNodelet(){};
    ~ObCameraNodelet(){}

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
        viewer_.reset(new OBCameraNodeFactory(nh, pnh));

    }
private:
    boost::shared_ptr<OBCameraNodeFactory> viewer_;
};

} // namespace astra_camera

PLUGINLIB_EXPORT_CLASS(astra_camera::ObCameraNodelet, nodelet::Nodelet);