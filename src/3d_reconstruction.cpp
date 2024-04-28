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
 * 
 * Pointclouds-related:
 * http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29
 * http://docs.ros.org/en/melodic/api/tf2_ros/html/c++/
 * http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
 * https://pointclouds.org/documentation/singletonpcl_1_1_point_cloud.html#details
 * https://pointclouds.org/documentation/tutorials/matrix_transform.html
 * https://answers.ros.org/question/362734/how-to-get-the-transformation-matrix-from-translation-and-rotation/
 * 
 * Approximate-time synchronizer:
 * https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h> 
#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Range.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/// @brief Class to store image transform map and publish images 
class ImageTransformer{
public:
    /// @brief Constructor: get parameters from node handle
    /// @param nh Node handle for current node
    /// @param result Boolean reference signifying whether all the parameters are available 
    ImageTransformer(ros::NodeHandle* nh, bool& _result):
        it(*nh), tf_listener(tf_buffer)
    {
        // Get parameters: https://stackoverflow.com/a/31068195
        bool result =
            nh->getParam("rect_topic", rect_topic) &
            nh->getParam("edge_topic", edge_topic) &
            nh->getParam("pcl_topic", pcl_topic) &
            nh->getParam("sonar_frame", sonar_frame) &
            nh->getParam("world_frame", world_frame) &
            nh->getParam("threshold", threshold) &
            nh->getParam("min_area", min_area) &
            nh->getParam("min_ratio", min_ratio) &
            nh->getParam("min_range", min_range) &
            nh->getParam("max_range", max_range) &
            nh->getParam("horizontal_fov", horizontal_fov) &
            nh->getParam("vertical_fov", vertical_fov);
            nh->param<std::string>("floor_topic", floor_topic, "");
            nh->param<float>("floor_margin", floor_margin, 0.0);
        
        // Initialize publisher, subscriber and maps
        if (result)
        {
            // Subscribe & publicize
            floor_enabled = !floor_topic.empty();
            if (!floor_enabled)
                sub = it.subscribe(rect_topic, 1, &ImageTransformer::callback, this);
            else {
                image_sub.subscribe(*nh, rect_topic, 10);
                range_sub.subscribe(*nh, floor_topic, 10);
                time_sync.reset(new Sync(sync_policy(10), image_sub, range_sub));      
                time_sync->registerCallback(boost::bind(&ImageTransformer::callback_sync, this, _1, _2));
            }
            pub_img = it.advertise(edge_topic, 1);
            pub_pcl = nh->advertise<sensor_msgs::PointCloud2>(pcl_topic, 5);

        }
        _result = result;
    }

    /// @brief Callback function to process image
    /// @param msg Sonar rectangle image message 
    void callback(const sensor_msgs::ImageConstPtr& msg) {
        // Get volatile floor value
        float _floor = 0;
        if (floor_enabled) _floor = floor;

        // Try to convert to cv - this is dependent on the process on sonar side
        cv_bridge::CvImagePtr cv_ptr;
        ros::Time msg_time;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
            beam_count = cv_ptr->image.cols;
            bin_count = cv_ptr->image.rows;
            msg_time = ros::Time(msg->header.stamp.sec, msg->header.stamp.nsec);
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
            float contour_ratio = bbox.height * 1.0 / bbox.width;
            if (contour_area > min_area && contour_ratio > min_ratio)
                large_contours.push_back(contours[i]);
        }
        // Draw the largest contours & detect the topmost edge
        std::vector<std::vector<cv::Point>> edge_contours(1, std::vector<cv::Point>());
        cv::Mat thick_borders = cv::Mat::zeros(bin_count, beam_count, CV_8UC1);
        cv::drawContours(thick_borders, large_contours, -1, 255, 2);
        std::vector<int> edges = std::vector<int>(beam_count, bin_count);
        float skip_bins = bin_count * min_range / max_range;
        for (int j = 0; j < beam_count; j++)
            for (int i = (int)skip_bins; i < bin_count; i++) {
                if (thick_borders.at<unsigned char>(i, j) > 0) {
                    edges[j] = i;
                    edge_contours[0].push_back(cv::Point(j, i));
                    break;
                }
            }

        // Draw final edge on image
        cv::Mat bg_channels[3] = {
            cv_ptr->image, cv_ptr->image, cv_ptr->image
        };
        cv::Mat fg_channels[3] = {
            cv_ptr->image + dest, cv_ptr->image + dest, cv_ptr->image
        };
        cv::Mat bg, fg;
        cv::merge(bg_channels, 3, bg);
        cv::merge(fg_channels, 3, fg);
        cv::addWeighted(bg, 0.5, fg, 0.5, 0, cv_ptr->image);
        cv_ptr->encoding = "bgr8";
        cv::polylines(cv_ptr->image, edge_contours, false, cv::Scalar(255, 255, 255));

        // Publish image
        pub_img.publish(cv_ptr->toImageMsg());

        // Listen to transform at the sonar's time
        geometry_msgs::TransformStamped transform;
        try{
            transform = tf_buffer.lookupTransform(world_frame, sonar_frame, msg_time);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Publish the point cloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud, tmp_cloud_1;
        for (int i = 0; i < beam_count; i++) {
            if (edges[i] != bin_count) {
                float theta = horizontal_fov * ((float)i / beam_count - 0.5);
                float phi = -vertical_fov / 2;
                float range = edges[i] * max_range / bin_count;
                pcl_cloud.push_back(pcl::PointXYZ(
                    range * cos(phi) * cos(theta),
                    range * cos(phi) * sin(theta),
                    range * sin(phi)
                ));
            }
        }
        // Move to world frame & convert to PointCloud2
        Eigen::Affine3d tf_matrix = tf2::transformToEigen(transform.transform);
        pcl::transformPointCloud(pcl_cloud, tmp_cloud_1, tf_matrix);

        // Get global floor value
        if (floor_enabled) {
            Eigen::Vector3f floor_vector = Eigen::Affine3f(tf_matrix) * Eigen::Vector3f(0, 0, -_floor);
            float z = floor_vector[2], margin = abs(z) * floor_margin;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = tmp_cloud_1.makeShared();
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud_in);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(z - margin, z + margin);
            pass.filter(tmp_cloud_1);
        }

        pcl::PCLPointCloud2 tmp_cloud_2;
        pcl::toPCLPointCloud2(tmp_cloud_1, tmp_cloud_2);

        // Convert to ROS message & publish
        sensor_msgs::PointCloud2 ros_cloud;
        pcl_conversions::fromPCL(tmp_cloud_2, ros_cloud);
        ros_cloud.header.frame_id = world_frame;
        ros_cloud.header.stamp = msg_time;
        pub_pcl.publish(ros_cloud);
    }

    /// @brief Callback function to process both image and range
    /// @param image_msg Sonar rectangle image message 
    /// @param range_msg Floor range message 
    void callback_sync(
        const sensor_msgs::ImageConstPtr& image_msg,
        const sensor_msgs::RangeConstPtr& range_msg
    ) {
        floor = range_msg->range;
        callback(image_msg);
    }

public:
    // Parameters
    std::string rect_topic, edge_topic, pcl_topic, sonar_frame, world_frame;
    float threshold, min_area, min_ratio, min_range, max_range;
    int beam_count, bin_count;
    float horizontal_fov, vertical_fov;

    // Image transport variables
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    image_transport::Publisher pub_img;

    // Point cloud variables
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    ros::Publisher pub_pcl;

    // (Optional) rangefinder variables
    std::string floor_topic;
    float floor_margin;
    bool floor_enabled;
    volatile float floor;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Range> sync_policy;
    typedef message_filters::Synchronizer<sync_policy> Sync;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::Range> range_sub;
    boost::shared_ptr<Sync> time_sync;
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
        ROS_ERROR("[3d_reconstruction] Some parameters could not be found");
        return 0;
    }

    ros::spin();

    return 0;
}