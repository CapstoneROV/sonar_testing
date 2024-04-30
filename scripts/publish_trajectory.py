#!/usr/bin/env python
import json
import rospy
import tf2_ros
import tf.transformations
from geometry_msgs.msg import TransformStamped

def interpolate_pose(pose1, pose2, ratio):
    # Interpolate position
    x = (1 - ratio) * pose1[0] + ratio * pose2[0]
    y = (1 - ratio) * pose1[1] + ratio * pose2[1]
    z = (1 - ratio) * pose1[2] + ratio * pose2[2]

    # Convert RPY to quaternions
    quat1 = tf.transformations.quaternion_from_euler(
        pose1[3],
        pose1[4],
        pose1[5]
    )
    quat2 = tf.transformations.quaternion_from_euler(
        pose2[3],
        pose2[4],
        pose2[5]
    )

    # Interpolate quaternion
    quat = tf.transformations.quaternion_slerp(quat1, quat2, ratio)
    return [x, y, z, quat[0], quat[1], quat[2], quat[3]]

def publish_transforms(json_file):
    # Read JSON file
    with open(json_file, 'r') as f:
        data = json.load(f)

    # Initialize ROS node
    rospy.init_node('pose_publisher')
    br = tf2_ros.TransformBroadcaster()

    # Iterate through poses and publish each as a transform
    intervals = 20
    delay = 0
    for i in range(len(data['times']) - 1):
        if rospy.is_shutdown():
            break
        time = rospy.Time(data['times'][i][0], data['times'][i][1]) - rospy.Duration(delay)
        next_time = rospy.Time(data['times'][i + 1][0], data['times'][i + 1][1]) - rospy.Duration(delay)
        step = (next_time - time) / intervals
        for j in range(intervals):
            while (not rospy.is_shutdown()) and rospy.Time.now() < time - rospy.Duration(1):
                rospy.sleep(0.1)
            if rospy.is_shutdown():
                break
            
            pose = interpolate_pose(data['poses'][i], data['poses'][i + 1], float(j) / intervals)
            t = TransformStamped()
            
            t.header.stamp = time
            t.header.frame_id = 'map_ned'
            t.child_frame_id = 'dummy_link'
            
            t.transform.translation.x = pose[0]
            t.transform.translation.y = pose[1]
            t.transform.translation.z = pose[2]
            
            t.transform.rotation.x = pose[3]
            t.transform.rotation.y = pose[4]
            t.transform.rotation.z = pose[5]
            t.transform.rotation.w = pose[6]
            
            br.sendTransform(t)
            time += step
            
    # Non-interpolating
    # for time, pose in zip(data['times'], data['poses']):
    #     t = TransformStamped()
        
    #     t.header.stamp = rospy.Time(time[0], time[1]) + rospy.Duration(0)
    #     t.header.frame_id = 'map_ned'
    #     t.child_frame_id = 'dummy_link'
        
    #     t.transform.translation.x = pose[0]
    #     t.transform.translation.y = pose[1]
    #     t.transform.translation.z = pose[2]
        
    #     quat = tf.transformations.quaternion_from_euler(
    #         pose[3],
    #         pose[4],
    #         pose[5],
    #     )
        
    #     t.transform.rotation.x = quat[0]
    #     t.transform.rotation.y = quat[1]
    #     t.transform.rotation.z = quat[2]
    #     t.transform.rotation.w = quat[3]
        
    #     br.sendTransform(t)
    #     while rospy.Time.now() < rospy.Time(time[0], time[1]) - rospy.Duration(1):
    #         rospy.sleep(1)
            
if __name__ == '__main__':
    try:
        publish_transforms('/home/ardupilot/capstonerov/src/sonar_testing/scripts/trajectory.json')
    except rospy.ROSInterruptException:
        pass
