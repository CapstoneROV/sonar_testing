/**
 * This ROS node transforms sonar fan images to rectangular images.
 * http://wiki.ros.org/roscpp/Overview/Parameter%20Server
 * http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown
 * http://wiki.ros.org/image_transport
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/// @brief Class to store image transform map and publish images 
class ImageTransformer{
public:
    /// @brief Constructor: get parameters from node handle
    /// @param nh Node handle for current node
    /// @param result Boolean reference signifying whether all the parameters are available 
    ImageTransformer(ros::NodeHandle* nh, bool& _result):
        it(*nh)
    {
        // Get parameters: https://stackoverflow.com/a/31068195
        bool result =
            nh->getParam("fan_topic", fan_topic) &
            nh->getParam("rect_topic", rect_topic) &
            nh->getParam("origin_x", origin_x) &
            nh->getParam("origin_y", origin_y) &
            nh->getParam("top_x", top_x) &
            nh->getParam("top_y", top_y);
        
        // Initialize publisher, subscriber and maps
        if (result)
        {
            // Subscribe & publicize
            sub = it.subscribe(fan_topic, 1, &ImageTransformer::callback, this);
            pub = it.advertise(rect_topic, 1);
        }
        
        _result = result;
    }

    /// @brief Callback function to process image
    /// @param msg Sonar fan image message 
    void callback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        // Try to convert to cv - this is dependent on the process on sonar side
        try
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("[sonar_transformer] cv_bridge exception: %s", e.what());
            return;
        }
    }

public:
    // Parameters
    std::string fan_topic, rect_topic;
    float origin_x, origin_y, top_x, top_y;
    int beam_count, bin_count;

    // Image transport variables
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;
};

int main(int argc, char **argv)
{
    // Setup node
    ros::init(argc, argv, "sonar_transformer");
    ros::NodeHandle nh("~");
    
    // Create class & get parameters
    bool params_available;
    ImageTransformer tfm(&nh, params_available);
    if (!params_available) {
        ROS_ERROR("[sonar_transformer] Some parameters could not be found");
        return 0;
    }

    ros::spin();

    return 0;
}