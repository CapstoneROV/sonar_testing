#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, tf2_ros::Buffer *tfBuffer, ros::Publisher* pub) {
    sensor_msgs::PointCloud2 cloud_transformed;
    try {
        // Look up the transformation from ping360_depth_link to map
        geometry_msgs::TransformStamped transformStamped = 
            tfBuffer->lookupTransform("map", "ping360_depth_link", cloud_msg->header.stamp, ros::Duration(3.0));

        // Perform the transformation using tf2
        tf2::doTransform(*cloud_msg, cloud_transformed, transformStamped);

        // Publish the transformed point cloud
        pub->publish(cloud_transformed);
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_transformer");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/transformed_points", 1);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/dummy/ping360_sim_depth/depth/points", 1, 
                                                                 boost::bind(&cloudCallback, _1, &tfBuffer, &pub));

    ros::spin();
    return 0;
}
