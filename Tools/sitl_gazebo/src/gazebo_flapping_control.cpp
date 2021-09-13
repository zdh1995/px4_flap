/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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


#include "gazebo_flapping_control.h"

namespace gazebo {

GazeboFlappingControl::~GazeboFlappingControl() {
  updateConnection->~Connection();

  rho = 1.295;
  area = 1.0;
  alpha0 = 0.0;
  alpha_right = 0.0;
  alpha_left = 0.0;
  force_last_time = ignition::math::Vector3d(0, 0, 0);

}

void GazeboFlappingControl::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  GZ_ASSERT(_model, "FlappingControl _model pointer is NULL");
  GZ_ASSERT(_sdf, "FlappingControl _sdf pointer is NULL");
  model = _model;
  sdf = _sdf;

  world = model->GetWorld();
  GZ_ASSERT(world, "FlappingControl world pointer is NULL");

#if GAZEBO_MAJOR_VERSION >= 9
  physics = world->Physics();
#else
  physics = world->GetPhysicsEngine();
#endif
  GZ_ASSERT(physics, "FlappingControl physics pointer is NULL");

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("leftwing")){   //左翼关节
      std::string leftwingjoint = _sdf->Get<std::string>("leftwing");
      joint_[0] = model->GetJoint(leftwingjoint);
      if (joint_[0] == nullptr)
      {
        gzerr << "Joint with name[" << leftwingjoint << "] does not exist.\n";
      }
  }
  if (_sdf->HasElement("rightwing")){  //右翼关节
      std::string rightwingjoint = _sdf->Get<std::string>("rightwing");
      joint_[1] = model->GetJoint(rightwingjoint);
      if (joint_[1] == nullptr)
      {
        gzerr << "Joint with name[" << rightwingjoint << "] does not exist.\n";
      }
  }
  if (_sdf->HasElement("amplitude"))   //扑翼角幅度
      amp = _sdf->Get<double>("amplitude");
  else
  {
      gzerr << "need amp";
  }

  if (_sdf->HasElement("twist"))
    twist = _sdf->Get<double>("twist");//扭转角幅度

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    forward = _sdf->Get<ignition::math::Vector3d>("forward");//前向向量
  forward.Normalize();

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    upward = _sdf->Get<ignition::math::Vector3d>("upward");//向上向量
  upward.Normalize();


  if (_sdf->HasElement("a0"))
    alpha0 = _sdf->Get<double>("a0");//初始迎角

  if (_sdf->HasElement("radial_symmetry"))
    radialSymmetry = _sdf->Get<bool>("radial_symmetry");//径向对称

  if (_sdf->HasElement("wingspan"))
    wingspan = _sdf->Get<double>("wingspan");//翼展

  if (_sdf->HasElement("chord"))
    chord = _sdf->Get<double>("chord");//翼根弦长

  if (_sdf->HasElement("frequency"))
    frequency = _sdf->Get<double>("frequency");//扑翼频率

  if (_sdf->HasElement("rho"))
    rho = _sdf->Get<double>("rho");//空气密度

  if (_sdf->HasElement("lcp"))
    lcp = _sdf->Get<ignition::math::Vector3d>("lcp");//本体坐标系中的压力中心
  if (_sdf->HasElement("rcp"))
    rcp = _sdf->Get<ignition::math::Vector3d>("rcp");//本体坐标系中的压力中心

  if (_sdf->HasElement("link_name"))//指向当前链接关节对应目标（即机体）的指针
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    link = model->GetLink(linkName);
    // GZ_ASSERT(link, "Link was NULL");

    if (!link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The FlappingControl will not generate forces\n";
    }
  }

  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);


  // Listen to the update event. This event is broadcast every
  // simulation iteration.

  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboFlappingControl::OnUpdate, this, _1));

  //订阅消息
  command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>("~/" + model->GetName() + command_sub_topic_, &GazeboFlappingControl::VelocityCallback, this);

  //常数
  pi = M_PI;
  k = 4.4;
  e = 0.8;
  uu = 17.9 * 0.000001; //空气动力粘性系数单位：Ps*s

  //预先处理

  aspect_ratio = pow(wingspan,2)/(pi*chord*0.25*wingspan);

  c1 = 0.5*aspect_ratio/(2.32+aspect_ratio);

  c2 = 0.181+0.772/aspect_ratio;

  cf = 1-c1*k*k/(k*k+c2*c2);

  cg = c1*c2*k/(k*k+c2*c2);

  ck = pow((cf*cf+cg*cg),0.5);

  //打开输出文件
  outputfile.open ("frequency.txt");
  dFoutput.open("dFout.txt");
  pure_motor_vel = 0;

 first_in = true;
  // Create the first order filter.
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(double(1.0 / 80.0), double(1.0 / 40.0), pure_motor_vel));
}

// This gets called by the world update start event.
void GazeboFlappingControl::OnUpdate(const common::UpdateInfo& /*_info*/) {

//  if(!received_first_referenc_)
//    return;
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world->SimTime();
#else
  common::Time now = world->GetSimTime();
#endif


frequency = pure_motor_vel / 290;


if ((frequency < 1) && (first_in)) {
  //  outputfile << "return！" << frequency <<std::endl;
    return;
}

if (toss){
     toss = false;
     model->SetLinearVel({6, 0, 1});
     prev_sim_time_ = now.Double();
     first_in = false;
}

sampling_time_ = now.Double() - prev_sim_time_;

double sinamp = amp*sin(2*pi*frequency*sampling_time_);


  for (int j = 0; j < 2; j++)
  {
      joint_[j]->SetPosition(0,sinamp*(j-0.5)*2);
  }

  GZ_ASSERT(link, "Link was NULL");

  //系数

  double beta = 0.5*amp*cos(2*frequency*pi*sampling_time_);

  double beta1 = -2*frequency*pi*0.5*amp*sin(2*frequency*pi*sampling_time_);

  double beta2 = -2*frequency*pi*2*frequency*pi*0.5*amp*cos(2*frequency*pi*sampling_time_);

  ignition::math::Vector3d accel = link->RelativeLinearAccel();

  ignition::math::Vector3d cp = ignition::math::Vector3d(-0.05,0,0);


  //获得机体的位置
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = link->WorldPose();
#else
  ignition::math::Pose3d pose = ignitionFromGazeboMath(link->GetWorldPose());
#endif

  //将向前和向上矢量旋转进入惯性系
  ignition::math::Vector3d forwardI = pose.Rot().RotateVector(forward);
  //forwardI= forward;

  ignition::math::Vector3d upwardI;
  if (radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
  }
  else
  {
    upwardI = pose.Rot().RotateVector(ignition::math::Vector3d(0, 0, 1));

  }
  ignition::math::Vector3d leftI = pose.Rot().RotateVector(ignition::math::Vector3d(0, 1, 0));
  //惯性系中垂直于升阻面的矢量
  ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();

  const double minRatio = -1.0;
  const double maxRatio = 1.0;

  //未计算侧滑角
 // double sinSweepAngle = ignition::math::clamp(
  //    spanwiseI.Dot(velI), minRatio, maxRatio);

  //获得世界坐标系下左翼压力中心处的速度
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel_left = link->WorldLinearVel(lcp);
#else
  ignition::math::Vector3d vel_left = ignitionFromGazeboMath(link->GetWorldLinearVel(cp));
#endif
  ignition::math::Vector3d velI_left = vel_left;
  velI_left.Normalize();
  //获得世界坐标系下右翼压力中心处的速度
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel_right = link->WorldLinearVel(rcp);
#else
  ignition::math::Vector3d vel_right = ignitionFromGazeboMath(link->GetWorldLinearVel(cp));
#endif
  ignition::math::Vector3d velI_right = vel_right;
  velI_right.Normalize();

  //去除升阻面外的速度分量，计算升力、阻力、力矩方向向量
  ignition::math::Vector3d velInLDPlane_left = vel_left - vel_left.Dot(spanwiseI)*velI_left;
  ignition::math::Vector3d velInLDPlane_right = vel_right - vel_right.Dot(spanwiseI)*velI_right;


  double speedInLDPlane_left = velInLDPlane_left.Length();
  double speedInLDPlane_right = velInLDPlane_right.Length();

  ignition::math::Vector3d dragDirection_left = -velInLDPlane_left;
  dragDirection_left.Normalize();
  ignition::math::Vector3d dragDirection_right = -velInLDPlane_right;
  dragDirection_right.Normalize();

  ignition::math::Vector3d liftI_left = spanwiseI.Cross(velInLDPlane_left);
  liftI_left.Normalize();
  ignition::math::Vector3d liftI_right = spanwiseI.Cross(velInLDPlane_right);
  liftI_right.Normalize();

  ignition::math::Vector3d momentDirection = spanwiseI;

  //计算速度迎角
  double cosAlpha = ignition::math::clamp(liftI_left.Dot(upwardI), minRatio, maxRatio);

  if (liftI_left.Dot(forwardI) >= 0.0)
    alpha_left = alpha0 + acos(cosAlpha);
  else
    alpha_left = alpha0 - acos(cosAlpha);

  while (fabs(alpha_left) > 0.5 * M_PI)
    alpha_left = alpha_left > 0 ? alpha_left - M_PI
                                  : alpha_left + M_PI;
  //right
  cosAlpha = ignition::math::clamp(liftI_right.Dot(upwardI), minRatio, maxRatio);

  if (liftI_right.Dot(forwardI) >= 0.0)
    alpha_right = alpha0 + acos(cosAlpha);
  else
    alpha_right = alpha0 - acos(cosAlpha);

  while (fabs(alpha_right) > 0.5 * M_PI)
    alpha_right = alpha_right > 0 ? alpha_right - M_PI
                                  : alpha_right + M_PI;


  //处理每个剖面的速度

  double chordx , vforward_left , vupward_left , vforward_right , vupward_right , theta , theta1 , theta2 , spanx ;

  double  vrel_left , psi_left, alphaeff_left , vrel_right , psi_right, alphaeff_right;

  double Reref_left , clc_left ,cdi_left ,cff_left ,cdp_left;

  double dLc_left, dNnc_left, dDp_left, dDi_left, dDd_left, dFx_left, dFy_left, dFL_left, dFT_left, FL_left, FT_left;

  double Reref_right , clc_right ,cdi_right ,cff_right ,cdp_right;

  double dLc_right, dNnc_right, dDp_right, dDi_right, dDd_right, dFx_right, dFy_right, dFL_right, dFT_right, FL_right, FT_right;


  int n = 200;

  FT_left = 0;
  FL_left = 0;
  FT_right = 0;
  FL_right = 0;


  for (int i=0 ; i<n ; i++)
  {
      chordx = pow((1-pow(i*0.005,2)),0.5)*chord;

      spanx = 0.5*wingspan*i/n;

      theta = 0.5*twist*cos(2*frequency*pi*sampling_time_+pi/2)*i/n;

      theta1 = -2*frequency*pi*0.5*twist*sin(2*frequency*pi*sampling_time_+pi/2)*i/n;

      theta2 = -2*frequency*pi*2*frequency*pi*0.5*twist*cos(2*frequency*pi*sampling_time_+pi/2)*i/n;

      vforward_left = speedInLDPlane_left*cos(alpha_left) + 0.75*chordx*theta1*sin(theta);

      vupward_left = speedInLDPlane_left*sin(alpha_left) - beta1*cos(beta)*spanx+0.75*chordx*theta1*cos(theta);

      vforward_right = speedInLDPlane_right*cos(alpha_right) + 0.75*chordx*theta1*sin(theta);

      vupward_right = speedInLDPlane_right*sin(alpha_right) - beta1*cos(beta)*spanx+0.75*chordx*theta1*cos(theta);

      vrel_left = pow(pow(vforward_left,2)+pow(vupward_left,2) , 0.5);  //剖面速度

      psi_left = atan(vupward_left / vforward_left);

      alphaeff_left = theta + psi_left; //实际迎角

      vrel_right = pow(pow(vforward_right,2)+pow(vupward_right,2) , 0.5);  //剖面速度

      psi_right = atan(vupward_right / vforward_right);

      alphaeff_right = theta + psi_right; //实际迎角


      //计算气动力

      Reref_left = rho*speedInLDPlane_left*chord/uu;

      clc_left = 2*pi*ck*sin(alphaeff_left);
      cdi_left = clc_left*clc_left/(e*pi*aspect_ratio);
      cff_left = 0.445*pow(log10(Reref_left),-2.58);
      cdp_left = cff_left*k;

      dLc_left = 0.5*rho*vrel_left*vrel_left*clc_left*chordx*0.5*wingspan/n; //截面升力

      dNnc_left = -rho*pi*chordx*chordx/4*(theta1*speedInLDPlane_left+beta2*spanx*cos(theta)-0.5*theta2)*0.5*wingspan/n; //附加质量力

      dDp_left = 0.5*rho*vrel_left*vrel_left*cdp_left*chordx*0.5*wingspan/n;
      dDi_left = 0.5*rho*vrel_left*vrel_left*cdi_left*chordx*0.5*wingspan/n;

      dDd_left = dDp_left+dDi_left;

      dFx_left=-dLc_left*sin(psi_left)+dDd_left*cos(psi_left);
      dFy_left=dLc_left*cos(psi_left)+dDd_left*sin(psi_left);

      dFL_left = dFy_left*cos(alpha_left)-dFx_left*sin(alpha_left)+dNnc_left*cos(-theta)*cos(beta)*cos((alpha_left));
      dFT_left = dFy_left*sin(alpha_left)+dFx_left*cos(alpha_left)-dNnc_left*sin(-theta)*sin(beta);

      FT_left = FT_left + dFT_left;
      FL_left = FL_left + dFL_left;
      //计算气动力

      Reref_right = rho*speedInLDPlane_right*chord/uu;

      clc_right = 2*pi*ck*sin(alphaeff_right);
      cdi_right = clc_right*clc_right/(e*pi*aspect_ratio);
      cff_right = 0.445*pow(log10(Reref_right),-2.58);
      cdp_right = cff_right*k;

      dLc_right = 0.5*rho*vrel_right*vrel_right*clc_right*chordx*0.5*wingspan/n; //截面升力

      dNnc_right = -rho*pi*chordx*chordx/4*(theta1*speedInLDPlane_right+beta2*spanx*cos(theta)-0.5*theta2)*0.5*wingspan/n; //附加质量力

      dDp_right = 0.5*rho*vrel_right*vrel_right*cdp_right*chordx*0.5*wingspan/n;
      dDi_right = 0.5*rho*vrel_right*vrel_right*cdi_right*chordx*0.5*wingspan/n;

      dDd_right = dDp_right+dDi_right;

      dFx_right=-dLc_right*sin(psi_right)+dDd_right*cos(psi_right);
      dFy_right=dLc_right*cos(psi_right)+dDd_right*sin(psi_right);

      dFL_right = dFy_right*cos(alpha_right)-dFx_right*sin(alpha_right)+dNnc_right*cos(-theta)*cos(beta)*cos((alpha_right));
      dFT_right = dFy_right*sin(alpha_right)+dFx_right*cos(alpha_right)-dNnc_right*sin(-theta)*sin(beta);

      FT_right = FT_right + dFT_right;
      FL_right = FL_right + dFL_right;
      dFoutput << FT_right << " ";
  }
 dFoutput << sampling_time_ <<std::endl;

  // compute lift force at cp
  ignition::math::Vector3d lift_left = FL_left * liftI_left;
  //ignition::math::Vector3d lift = FL * ignition::math::Vector3d(0, 0, 1);

  // drag at cp
  ignition::math::Vector3d drag_left = FT_left * dragDirection_left;
  //ignition::math::Vector3d drag = FT * ignition::math::Vector3d(-1, 0, 0);

  ignition::math::Vector3d force_left = lift_left + drag_left;

  // compute lift force at cp
  ignition::math::Vector3d lift_right = FL_right * liftI_right;
  //ignition::math::Vector3d lift = FL * ignition::math::Vector3d(0, 0, 1);

  // drag at cp
  ignition::math::Vector3d drag_right = FT_right * dragDirection_right;
  //ignition::math::Vector3d drag = FT * ignition::math::Vector3d(-1, 0, 0);

  ignition::math::Vector3d force_right = lift_right + drag_right;

  //force_right = ignition::math::Vector3d(force_right.X(),-force_right.Y(),force_right.Z());


  force_left.Correct();
  force_right.Correct();
  lcp.Correct();
  rcp.Correct();
  // apply forces at cg (with torques for position shift)


  ignition::math::Vector3d gravity = world->Gravity()* (link->GetInertial()->Mass()+0.015);

  ignition::math::Pose3d center = link->WorldCoGPose();

  ignition::math::Vector3d attitude = center.Rot().Euler();

 //link->AddForceAtRelativePosition(force_left,lcp);
 //link->AddForceAtRelativePosition(force_right,rcp);
  ignition::math::Vector3d force_all = ignition::math::Vector3d(-FT_left-FT_right,0,FL_left+FL_right)/ 0.9491;
  //ignition::math::Vector3d force_all = ignition::math::Vector3d(2,0,0.5) / 0.9491;
  force_all.Correct();

  link->AddRelativeForce(force_all);

  ignition::math::Vector3d torque = ignition::math::Vector3d((FL_left-FL_right)*0.1,(FL_left+FL_right)*0.05,(FT_left-FT_right)*0.1);
  //ignition::math::Vector3d torque = ignition::math::Vector3d((FL_left-FL_right)*0.1,0,(FT_left-FT_right)*0.1);

  torque.Correct();

  link->AddRelativeTorque(torque);

 //link->AddForce(-gravity);

  ignition::math::Vector3d moment = link->RelativeTorque();

  ignition::math::Vector3d ang_vel = link->RelativeAngularVel();

  ignition::math::Vector3d force_trans = link->RelativeForce();

  ignition::math::Vector3d vel = link->WorldLinearVel(cp);


  if (outputfile.is_open())
  {
     outputfile //<< liftI_left.Dot(upwardI) << " " << liftI_right.Dot(upwardI) << " "
                 << vel[0] << " " << vel[1] << " " << vel[2] << " "
                // << liftI_left[0] << " " << liftI_left[1] << " " << liftI_left[2] << " "
                 //   << liftI_right[0] << " " << liftI_right[1] << " " << liftI_right[2] << " "
                //    << vel_right[0] << " " << vel_right[1] << " " << vel_right[2] << " "
                  << speedInLDPlane_left << " " << speedInLDPlane_right << " "
                 //     << spanwiseI[0] << " " << spanwiseI[1] << " " << spanwiseI[2] << " "
                //          << upwardI[0] << " " << upwardI[1] << " " << upwardI[2] << " "
              //                << forwardI[0] << " " << forwardI[1] << " " << forwardI[2] << " "
                    << moment[0] << " " << moment[1] << " " << moment[2] << " "
                 << attitude[0] << " " << attitude[1] << " " << attitude[2] << " "
                 << FT_left<< " " << FT_right << " " << FL_left << " " << FL_right << " "
                 << alpha_left << " "  << alpha_right << " "<< frequency << " " <<sampling_time_ <<std::endl;
             //    <<  sampling_time_ <<std::endl;
  } else
  {
      gzdbg << "openfile failed!\n";
  }
  force_last_time = force_left;
}
void GazeboFlappingControl::VelocityCallback(CommandMotorSpeedPtr &motorspeed) {
  pure_motor_vel = static_cast<double>(motorspeed->motor_speed(motor_number_));

  received_first_referenc_ = true;

  if (first_in && pure_motor_vel > 200) toss = true;

  //滤波
  ref_motor_vel_ = rotor_velocity_filter_->updateFilter(pure_motor_vel, sampling_time_);
}


GZ_REGISTER_MODEL_PLUGIN(GazeboFlappingControl);
}
