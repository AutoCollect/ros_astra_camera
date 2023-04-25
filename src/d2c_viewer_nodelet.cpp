#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "astra_camera/d2c_viewer.h"


namespace astra_camera
{

/**
 * @brief Nodelet-wrapper of the BumpBlinkController class
 */
class D2cViewerNodelet : public nodelet::Nodelet
{
public:
    D2cViewerNodelet(){};
    ~D2cViewerNodelet(){}

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
        viewer_.reset(new D2CViewer(nh, pnh));

    }
private:
    boost::shared_ptr<D2CViewer> viewer_;
};

} // namespace astra_camera

PLUGINLIB_EXPORT_CLASS(astra_camera::D2cViewerNodelet, nodelet::Nodelet);