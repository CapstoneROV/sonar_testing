/**
 * This ROS node detects the topmost edge in rectangle sonar images.
 * http://wiki.ros.org/roscpp/Overview/Parameter%20Server
 * http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown
 * http://wiki.ros.org/image_transport
 * http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 * https://docs.opencv.org/3.4/d1/da0/tutorial_remap.html
 * 
 * Contours-related:
 * https://stackoverflow.com/questions/8449378/finding-contours-in-opencv
 * https://stackoverflow.com/questions/11239938/how-to-process-each-contours
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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
            nh->getParam("rect_topic", rect_topic) &
            nh->getParam("edge_topic", edge_topic) &
            nh->getParam("threshold", threshold) &
            nh->getParam("min_area", min_area) &
            nh->getParam("min_ratio", min_ratio) &
            nh->getParam("skip_bins", skip_bins) &
            nh->getParam("max_range", max_range) &
            nh->getParam("horizontal_fov", horizontal_fov);
        
        // Initialize publisher, subscriber and maps
        if (result)
        {
            // Subscribe & publicize
            sub = it.subscribe(rect_topic, 1, &ImageTransformer::callback, this);
            pub = it.advertise(edge_topic, 1);
        }
        
        _result = result;
    }

    /// @brief Callback function to process image
    /// @param msg Sonar rectangle image message 
    void callback(const sensor_msgs::ImageConstPtr& msg) {
        // Try to convert to cv - this is dependent on the process on sonar side
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
            beam_count = cv_ptr->image.cols;
            bin_count = cv_ptr->image.rows;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("[3d_reconstruction] cv_bridge exception: %s", e.what());
            return;
        }
        // Thresholding
        cv::Mat dest = cv::Mat(bin_count, beam_count, CV_8UC1);
        cv::threshold(cv_ptr->image, dest, threshold, 127, cv::THRESH_BINARY);

        // Find the largest contours in the image
        std::vector<std::vector<cv::Point>> contours, large_contours;
        cv::findContours(dest, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (size_t i = 0; i < contours.size(); i++) {
            // Get contour area
            float contour_area = cv::contourArea(contours[i]);

            // Get contour aspect ratio
            cv::Rect bbox = cv::boundingRect(contours[i]);
            float contour_ratio = bbox.width * 1.0 / bbox.height;
            if (contour_area > min_area && contour_ratio > min_ratio)
                large_contours.push_back(contours[i]);
        }

        // Draw the largest contours & detect the topmost edge
        std::vector<std::vector<cv::Point>> edge_contours(1, std::vector<cv::Point>());
        cv::Mat thick_borders = cv::Mat(bin_count, beam_count, CV_8UC1);
        thick_borders.setTo(0);
        cv::drawContours(thick_borders, large_contours, -1, 255, 2);
        // std::vector<int> edges = std::vector<int>(beam_count, bin_count);
        for (int j = 0; j < beam_count; j++)
            for (int i = skip_bins; i < bin_count; i++)
                if (thick_borders.at<unsigned char>(i, j) > 0) {
                    // edges[j] = i;
                    edge_contours[0].push_back(cv::Point(j, i));
                    // cv::circle(cv_ptr->image, cv::Point(j, i), 2, 255);
                    break;
                }
        
        // Draw final edge on image
        cv::polylines(dest, edge_contours, false, 255);
        cv_ptr->image = dest;

        // Publish image
        pub.publish(cv_ptr->toImageMsg());
    }

public:
    // Parameters
    std::string rect_topic, edge_topic;
    float threshold, min_area, min_ratio, max_range;
    int beam_count, bin_count, skip_bins;
    float horizontal_fov;

    // Image transport variables
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;
};

int main(int argc, char **argv)
{
    // Setup node
    ros::init(argc, argv, "3d_reconstruction");
    ros::NodeHandle nh("~");
    
    // Create class & get parameters
    bool params_available;
    ImageTransformer tfm(&nh, params_available);
    if (!params_available) {
        ROS_ERROR("[2] Some parameters could not be found");
        return 0;
    }

    ros::spin();

    return 0;
}