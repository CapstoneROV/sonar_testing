/**
 * This ROS node transforms sonar fan images to rectangular images.
 * http://wiki.ros.org/roscpp/Overview/Parameter%20Server
 * http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown
 * http://wiki.ros.org/image_transport
 * http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 * https://docs.opencv.org/3.4/d1/da0/tutorial_remap.html
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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
            nh->getParam("top_y", top_y) &
            nh->getParam("beam_count", beam_count) &
            nh->getParam("bin_count", bin_count) &
            nh->getParam("horizontal_fov", horizontal_fov);
        
        // Initialize publisher, subscriber and maps
        if (result)
        {
            // Subscribe & publicize
            sub = it.subscribe(fan_topic, 1, &ImageTransformer::callback, this);
            pub = it.advertise(rect_topic, 1);

            // Create maps
            map_x = cv::Mat(bin_count, beam_count, CV_32FC1);
            map_y = cv::Mat(bin_count, beam_count, CV_32FC1);
            
            for (int i = 0; i < beam_count; i++)
                for (int j = 0; j < bin_count; j++) {
                    float theta = i * horizontal_fov / beam_count - horizontal_fov / 2;
                    float r_max = sqrt(pow(top_x - origin_x, 2) + pow(top_y - origin_y, 2));
                    float radius = j * r_max / bin_count;
                    map_x.at<float>(j, i) = origin_x - radius * sin(theta);
                    map_y.at<float>(j, i) = origin_y - radius * cos(theta);
                }
        }
        
        _result = result;
    }

    /// @brief Callback function to process image
    /// @param msg Sonar fan image message 
    void callback(const sensor_msgs::ImageConstPtr& msg) {
        // Try to convert to cv - this is dependent on the process on sonar side
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("[sonar_transformer] cv_bridge exception: %s", e.what());
            return;
        }

        // Create resulting image
        dest = cv::Mat(map_x.size(), cv_ptr->image.type());
        remap(cv_ptr->image, dest, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
        cv_ptr->image = dest;

        // Publish transformed image
        pub.publish(cv_ptr->toImageMsg());
    }

public:
    // Parameters
    std::string fan_topic, rect_topic;
    float origin_x, origin_y, top_x, top_y;
    int beam_count, bin_count;
    float horizontal_fov;

    // Image transport variables
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;

    // Image transform variables
    cv::Mat dest, map_x, map_y;
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