/*
 * Copyright 2020 Zeng DongHong Tsinghua University.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <boost/bind.hpp>
#include <Eigen/Eigen>

#include <iostream>
#include <fstream>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "CommandMotorSpeed.pb.h"
#include "MotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include <ignition/math.hh>

#include <stdio.h>

#include "common.h"


namespace gazebo {
//订阅数据，暂时不需要
//typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
//typedef const boost::shared_ptr<const mav_msgs::msgs::MotorSpeed> MotorSpeedPtr;

// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandSubTopic = "/gazebo/command/motor_speed";


typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
// This just proxies the motor commands from command/motor_speed to the single motors via internal
// ConsPtr passing, such that the original commands don't have to go n_motors-times over the wire.
//static const std::string kDefaultMotorVelocityReferencePubTopic = "/gazebo/command/motor_speed";
//static const std::string kDefaultCommandMotorSpeedSubTopic = "/command/motor_speed";

class GazeboFlappingControl : public ModelPlugin {
 public:
  GazeboFlappingControl() : ModelPlugin(),
        received_first_referenc_(false),
        toss(false),
        namespace_(kDefaultNamespace),
        command_sub_topic_(kDefaultCommandSubTopic),
        motor_number_(0),
        sampling_time_(0.01),
        prev_sim_time_(0)
//        motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
//        command_motor_speed_sub_topic_(kDefaultCommandMotorSpeedSubTopic)
  {}
  ~GazeboFlappingControl();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);
  void VelocityCallback(CommandMotorSpeedPtr &motorspeed);

  transport::SubscriberPtr command_sub_;

  /// \brief Connection to World Update events.
  event::ConnectionPtr updateConnection;

  /// \brief Pointer to world.
  physics::WorldPtr world;

  /// \brief Pointer to physics engine.
  physics::PhysicsEnginePtr physics;

  /// \brief Pointer to model containing plugin.
  physics::ModelPtr model;

  /// \brief Pointer to joints of wing.
  physics::JointPtr joint_[2];

  physics::JointPtr joint;

  /// \brief Pointer to link currently targeted by mud joint.
  physics::LinkPtr link;

  /// \brief SDF for this plugin;
  sdf::ElementPtr sdf;

  ///临时电机信号
  protected: double pure_motor_vel;

  protected: double ref_motor_vel_;

  protected: int motor_number_;

  protected: double sampling_time_;

  protected: double prev_sim_time_;

  /// \brief flapping amplitude
  double amp;
  /// \brief Coefficient of Lift / alpha slope.
  /// Lift = C_L * q * S
  /// where q (dynamic pressure) = 0.5 * rho * v^2
  protected: double twist;

  /// \brief Coefficient of Drag / alpha slope.
  /// Drag = C_D * q * S
  /// where q (dynamic pressure) = 0.5 * rho * v^2
  protected: double wingspan;

  /// \brief Coefficient of Moment / alpha slope.
  /// Moment = C_M * q * S
  /// where q (dynamic pressure) = 0.5 * rho * v^2
  protected: double chord;

  /// \brief angle of attach when airfoil stalls
  protected: double frequency;

  /// \brief air density
  /// at 25 deg C it's about 1.1839 kg/m^3
  /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
  protected: double rho;

  /// \brief if the shape is aerodynamically radially symmetric about
  /// the forward direction. Defaults to false for wing shapes.
  /// If set to true, the upward direction is determined by the
  /// angle of attack.
  protected: bool radialSymmetry;

  /// \brief effective planeform surface area
  protected: double area;

  /// \brief angle of sweep
  protected: double sweep;

  /// \brief initial angle of attack
  protected: double alpha0;

  /// \brief angle of attack
  protected: double alpha_left;
  protected: double alpha_right;

  /// \brief aspect ratio
  protected: double aspect_ratio;

  /// \brief aspect ratio
  protected: double k;

  /// \brief aspect ratio
  protected: double e;

  /// \brief aspect ratio
  protected: double uu;

  /// \brief center of pressure in link local coordinates
  protected: ignition::math::Vector3d lcp;

  /// \brief center of pressure in link local coordinates
  protected: ignition::math::Vector3d rcp;

  /// \brief Normally, this is taken as a direction parallel to the chord
  /// of the airfoil in zero angle of attack forward flight.
  protected: ignition::math::Vector3d forward;

  /// \brief A vector in the lift/drag plane, perpendicular to the forward
  /// vector. Inflow velocity orthogonal to forward and upward vectors
  /// is considered flow in the wing sweep direction.
  protected: ignition::math::Vector3d upward;

 private:

  bool received_first_referenc_;
  bool toss;
  bool first_in;
  Eigen::VectorXd input_reference_;

  std::string namespace_;
  std::string command_sub_topic_;
 // std::string motor_velocity_reference_pub_topic_;
 // std::string command_motor_speed_sub_topic_;

  transport::NodePtr node_handle_;

  double c1;
  double c2;
  double cg;
  double cf;
  double ck;
  double pi ;
  std::ofstream outputfile;
  std::ofstream dFoutput;

  std::unique_ptr<FirstOrderFilter<double>>  rotor_velocity_filter_;

  //上一时刻垂直于纵向面的向量
  ignition::math::Vector3d force_last_time;
 // transport::PublisherPtr motor_velocity_reference_pub_;
 // transport::SubscriberPtr cmd_motor_sub_;

 // boost::thread callback_queue_thread_;
 // void QueueThread();
 // void CommandMotorCallback(CommandMotorSpeedPtr &input_reference_msg);
};
}
