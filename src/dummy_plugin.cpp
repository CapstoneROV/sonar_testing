// Controller plugin courtesy of:
// - Gazebo:
// http://classic.gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
// https://classic.gazebosim.org/tutorials?tut=set_velocity&cat=
// https://github.com/osrf/gazebo_tutorials/blob/master/set_velocity/examples/set_vel_plugin/src/SetLinkVelocityPlugin.cpp
// https://github.com/osrf/gazebo_tutorials/blob/master/set_velocity/examples/set_vel_plugin/include/pid_link.hh
// 
// - ROS:
// https://classic.gazebosim.org/tutorials?tut=guided_i6
// http://classic.gazebosim.org/tutorials?tut=ros_plugins&cat=connect_ros
// http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html


#ifndef _DUMMY_PLUGIN_HH_
#define _DUMMY_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <pid_link.hh>

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"

#include <vector>
#include <string>

namespace gazebo
{
  /// \brief A plugin to control Dummy.
  class DummyPlugin : public ModelPlugin
  {
    /// \brief a pointer to the model this plugin was loaded by
    public: physics::ModelPtr model;
    /// \brief a PID controller to control the link
    public: PIDController controller;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    /// \brief Constructor
    public: DummyPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load dummy_plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }
      // Print dummy message
      std::cout << "\nThe dummy_plugin is attached to model[" <<
        _model->GetName() << "]\n";
      this->model = _model;

      // Parse SDF for parameters
      ignition::math::Vector3d pid_params[6];
      ignition::math::Pose3d pose; 
      double max_force, max_torque;
      std::string link_name;
      _sdf = _sdf->GetFirstElement();
      do
      {
        std::string param = _sdf->GetName();
        if (param == "pid_x")
          _sdf->GetValue()->Get(pid_params[0]);
        else if (param == "pid_y")
          _sdf->GetValue()->Get(pid_params[1]);
        else if (param == "pid_z")
          _sdf->GetValue()->Get(pid_params[2]);
        else if (param == "pid_r")
          _sdf->GetValue()->Get(pid_params[3]);
        else if (param == "pid_p")
          _sdf->GetValue()->Get(pid_params[4]);
        else if (param == "pid_w")
          _sdf->GetValue()->Get(pid_params[5]);
        else if (param == "max_force")
          _sdf->GetValue()->Get(max_force);
        else if (param == "max_torque")
          _sdf->GetValue()->Get(max_torque);
        else if (param == "link_name")
          _sdf->GetValue()->Get(link_name);
        _sdf = _sdf->GetNextElement();
      } while (_sdf);

      // Start controller 
      this->controller.Start(
        _model->GetLink(link_name), pid_params, max_force, max_torque
      );

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(
            "/" + this->model->GetName() + "/goal",
            1,
            boost::bind(&DummyPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&DummyPlugin::QueueThread, this));
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::PoseStampedConstPtr &_msg)
    {
      this->controller.UpdateTargetPose(
        {_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z,
        _msg->pose.orientation.w, _msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z}
      );
      // this->SetVelocity(_msg->data);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(DummyPlugin)
}
#endif