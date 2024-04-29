/**
 * This ROS node removes the grid from a sonar image based on a template image.
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
#include <opencv2/photo.hpp>

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
            nh->getParam("src_topic", src_topic) &
            nh->getParam("dst_topic", dst_topic) &
            nh->getParam("mask_file", mask_file) &
            nh->getParam("mask_threshold", mask_threshold) &
            nh->getParam("mask_erode_size", mask_erode_size) &
            nh->getParam("blur_size", blur_size);
            nh->param<bool>("mbe_sim_time", mbe_sim_time, false);
            nh->param<float>("mbe_delay", mbe_delay, -1.0);
        
        // Initialize publisher, subscriber and maps
        if (result)
        {
            // Subscribe & publicize
            sub = it.subscribe(src_topic, 1, &ImageTransformer::callback, this);
            pub = it.advertise(dst_topic, 1);

            // Create mask
            mask = cv::imread(mask_file, cv::IMREAD_GRAYSCALE);
            cv::Mat element = cv::getStructuringElement(
                cv::MORPH_RECT,
                cv::Size(mask_erode_size, mask_erode_size),
                cv::Point(mask_erode_size / 2, mask_erode_size / 2)
            );
            cv::threshold(mask, mask, mask_threshold, 255, cv::THRESH_BINARY_INV);
            cv::erode(mask, mask, element);
            cv::bitwise_not(mask, mask_inv);
            cv::blur(
                mask, mask_blurred,
                cv::Size(blur_size, blur_size),
                cv::Point(blur_size / 2, blur_size / 2)
            );
            // cv::cvtColor(mask_blurred, mask_blurred, cv::COLOR_GRAY2RGB);
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
            cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("[sonar_transformer] cv_bridge exception: %s", e.what());
            return;
        }

        // Create resulting image
        cv::Mat bg = cv::Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type()),
                fg = cv::Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type()),
                tmp = cv::Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());
        cv::bitwise_and(cv_ptr->image, cv_ptr->image, fg, mask);
        cv::blur(
            fg, bg,
            cv::Size(blur_size, blur_size),
            cv::Point(blur_size / 2, blur_size / 2)
        );
        bg = bg * 255 / mask_blurred;
        cv::bitwise_and(bg, bg, tmp, mask_inv);
        cv::add(tmp, fg, cv_ptr->image);

        // Publish image with grid removed
        sensor_msgs::ImagePtr img_msg = cv_ptr->toImageMsg();
        if (mbe_sim_time) img_msg->header.stamp = ros::Time::now();
        if (mbe_delay > 0) img_msg->header.stamp -= ros::Duration(mbe_delay);
        pub.publish(img_msg);
    }

public:
    // Parameters
    std::string src_topic, dst_topic, mask_file;
    int mask_threshold, mask_erode_size, blur_size;
    bool mbe_sim_time;
    float mbe_delay;

    // Image transport variables
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;

    // Image transform variables
    cv::Mat dest, mask, mask_inv, mask_blurred;
};

int main(int argc, char **argv)
{
    // Setup node
    ros::init(argc, argv, "grid_remover");
    ros::NodeHandle nh("~");
    
    // Create class & get parameters
    bool params_available;
    ImageTransformer tfm(&nh, params_available);
    if (!params_available) {
        ROS_ERROR("[grid_remover] Some parameters could not be found");
        return 0;
    }

    ros::spin();

    return 0;
}