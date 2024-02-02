// PID controller for link positioning courtesy of:
// https://github.com/osrf/gazebo_tutorials/blob/master/set_velocity/examples/set_vel_plugin/include/pid_link.hh
//
// with many changes for position instead of velocity, and Gazebo 9 instead of 7 courtesy of:
// https://github.com/wuwushrek/sim_cf/issues/2
// https://github.com/wuwushrek/sim_cf/pull/3/commits/8c1db3c5c379b4a8cc82b79039fc1c5f6e082e88
// https://github.com/dji-m100-ros/dji_m100_controllers_gazebo/issues/1
// https://answers.ros.org/question/361021/i-try-to-catkin_make-with-gazebo-9-but-i-got-these-errors/
// https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Model.html
// https://osrf-distributions.s3.amazonaws.com/ign-math/api/1.0.0/classignition_1_1math_1_1Vector3.html
// https://gazebosim.org/api/math/6.10/classignition_1_1math_1_1Pose3.html

#include <gazebo/gazebo.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#ifndef GAZEBO_TUTORIALS_SET_VELOCITY_PID_LINK_HH_
#define GAZEBO_TUTORIALS_SET_VELOCITY_PID_LINK_HH_

using namespace gazebo;

//////////////////////////////////////////////////
// \brief use a PID controller to apply forces to achieve a target position
class PIDController
{
  public: PIDController()
    {
    }

  public: ~PIDController()
    {
      this->Stop();
    }

  public: void Start(
      physics::LinkPtr _link,
      const ignition::math::Vector3d* pid_params,
      double _maxForce, double _maxTorque)
    {
      this->Stop();
      this->link = _link;
      this->targetPose = _link->WorldPose();
      
      double linear_imax = 10000.0;
      double angular_imax = 10000.0;

      // Add a PID controller for each DoF
      for (int i = 0; i < 3; i++)
      {
        common::PID controller_translation(pid_params[i].X(), pid_params[i].Y(), pid_params[i].Z(),
        // common::PID controller_translation(linear_p, linear_i, linear_d,
            linear_imax, -linear_imax, _maxForce, -_maxForce);
        this->controllers.push_back(controller_translation);
      }
      for (int i = 3; i < 6; i++)
      {
        common::PID controller_rotation(pid_params[i].X(), pid_params[i].Y(), pid_params[i].Z(),
        // common::PID controller_rotation(angular_p, angular_i, angular_d,
            angular_imax, -angular_imax, _maxTorque, -_maxTorque);
        this->controllers.push_back(controller_rotation);
      }

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PIDController::Update, this,
          std::placeholders::_1));
    }

  public: void Stop()
    {
      this->lastSimTime.Set(0.0);
      this->link.reset();
      this->controllers.clear();
      this->updateConnection.reset();
    }

  public: void Update(const common::UpdateInfo &_info)
  {
    common::Time curTime = _info.simTime;
    if (this->lastSimTime.Double() < 0.000001)
    {
      // First update, need a second one to get change time
    }
    if (curTime < this->lastSimTime)
    {
      // Time moved backwards (World reset?)
      this->Stop();
    }
    else
    {
      // Get change in time between updates
      double dt = (curTime - lastSimTime).Double();

      ignition::math::Pose3d curPose = this->link->WorldPose();
      ignition::math::Vector3d linearError = curPose.Pos() - this->targetPose.Pos();
      ignition::math::Vector3d angularError = (curPose.Rot() * this->targetPose.Rot().Inverse()).Euler();

      // Get forces to apply from controllers
      ignition::math::Vector3d worldForce;
      ignition::math::Vector3d worldTorque;
      worldForce.X(this->controllers[0].Update(linearError.X(), dt));
      worldForce.Y(this->controllers[1].Update(linearError.Y(), dt));
      worldForce.Z(this->controllers[2].Update(linearError.Z(), dt));
      worldTorque.X(this->controllers[3].Update(angularError.X(), dt));
      worldTorque.Y(this->controllers[4].Update(angularError.Y(), dt));
      worldTorque.Z(this->controllers[5].Update(angularError.Z(), dt));
      // double _pe, _ie, _de;
      // this->controllers[5].GetErrors(_pe, _ie, _de);
      // std::cout << _pe << ", " << _ie << ", " << _de << std::endl;

      // Add those forces to the body
      this->link->AddForce(worldForce);
      this->link->AddTorque(worldTorque);
    }
    this->lastSimTime = curTime;
  }

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  public: void UpdateTargetPose(ignition::math::Pose3d _pose)
  {
    targetPose = _pose;
  }

  private: physics::LinkPtr link;
  private: std::vector<common::PID> controllers;
  private: event::ConnectionPtr updateConnection;
  private: ignition::math::Pose3d targetPose;
  private: common::Time lastSimTime;
};

#endif