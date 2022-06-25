/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <algorithm>
#include <string>
#include <filesystem>

#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "liftdrag_plugin/liftdrag_plugin.h"
#include "liftdrag_plugin/dat_reader.h"
#include "liftdrag_plugin/lookup_table.h"

#include "Force.pb.h"

using namespace gazebo;
namespace fs = std::filesystem;

GZ_REGISTER_MODEL_PLUGIN(LiftDragPlugin)


/////////////////////////////////////////////////
LiftDragPlugin::LiftDragPlugin() : rho(1.2041)
{
  this->cp = ignition::math::Vector3d(0, 0, 0);
  this->forward = ignition::math::Vector3d(1, 0, 0);
  this->upward = ignition::math::Vector3d(0, 0, 1);
  this->wind_vel_ = ignition::math::Vector3d(0.0, 0.0, 0.0);
  this->area = 1.0;
  this->control_area  = 0;
  this->alpha0 = 0.0;
  this->alpha = 0.0;
  this->sweep = 0.0;
  this->velocityStall = 0.0;

  this->radialSymmetry = false;
}

/////////////////////////////////////////////////
LiftDragPlugin::~LiftDragPlugin()
{
}

/////////////////////////////////////////////////
void LiftDragPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LiftDragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDragPlugin world pointer is NULL");

#if GAZEBO_MAJOR_VERSION >= 9
  this->physics = this->world->Physics();
  this->last_pub_time = this->world->SimTime();
#else
  this->physics = this->world->GetPhysicsEngine();
  this->last_pub_time = this->world->GetSimTime();
#endif
  GZ_ASSERT(this->physics, "LiftDragPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("radial_symmetry"))
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry");

  if (_sdf->HasElement("a0"))
    this->alpha0 = _sdf->Get<double>("a0");

  if (_sdf->HasElement("cl_file")) {
      fs::path cl_path = _sdf->Get<std::string>("cl_file");
      gzdbg << "Path to C_L " << cl_path << std::endl;
      auto reader = DatReader<double>(cl_path);
      const auto& [indeces, values] = reader.read();
      this->cl_table = LookUpTable<double>(indeces, values);
  }

  if (_sdf->HasElement("cd_file")) {
    fs::path cl_path = _sdf->Get<std::string>("cd_file");
    auto reader = DatReader<double>(cl_path);
    const auto& [indeces, values] = reader.read();
    this->cd_table = LookUpTable<double>(indeces, values);
  }

  if (_sdf->HasElement("cm_file")) {
    fs::path cl_path = _sdf->Get<std::string>("cm_file");
    auto reader = DatReader<double>(cl_path);
    const auto& [indeces, values] = reader.read();
    this->cm_table = LookUpTable<double>(indeces, values);
  }

  if (_sdf->HasElement("control_cl_file")) {
      fs::path cl_path = _sdf->Get<std::string>("control_cl_file");
      auto reader = DatReader<double>(cl_path);
      const auto& [indeces, values] = reader.read();
      this->cl_flap_table = LookUpTable<double>(indeces, values);
  }

  if (_sdf->HasElement("control_cd_file")) {
    fs::path cd_path = _sdf->Get<std::string>("control_cd_file");
    auto reader = DatReader<double>(cd_path);
    const auto& [indeces, values] = reader.read();
    this->cd_flap_table = LookUpTable<double>(indeces, values);
  }

  if (_sdf->HasElement("control_cm_file")) {
    fs::path cm_path = _sdf->Get<std::string>("control_cm_file");
    auto reader = DatReader<double>(cm_path);
    const auto& [indeces, values] = reader.read();
    this->cm_flap_table = LookUpTable<double>(indeces, values);
  }

  if (_sdf->HasElement("cm_delta"))
    this->cm_delta = _sdf->Get<double>("cm_delta");

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<ignition::math::Vector3d>("cp");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
  this->forward.Normalize();

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
  this->upward.Normalize();

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("control_area"))
    this->control_area = _sdf->Get<double>("control_area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    // GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The LiftDragPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LiftDragPlugin::OnUpdate, this));
    }
  }

  if (_sdf->HasElement("robotNamespace"))
  {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_liftdrag_plugin] Please specify a robotNamespace.\n";
  }
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("topic_name")) {
      const auto lift_force_topic = this->sdf->Get<std::string>("topic_name");
      lift_force_pub_ = node_handle_->Advertise<physics_msgs::msgs::Force>("~/" + lift_force_topic);
      gzdbg << "Publishing to ~/" << lift_force_topic << std::endl;
  }

  if (_sdf->HasElement("windSubTopic")){
    this->wind_sub_topic_ = _sdf->Get<std::string>("windSubTopic");
    wind_sub_ = node_handle_->Subscribe("~/" + wind_sub_topic_, &LiftDragPlugin::WindVelocityCallback, this);
  }

  if (_sdf->HasElement("control_joint_name"))
  {
    std::string controlJointName = _sdf->Get<std::string>("control_joint_name");
    this->controlJoint = this->model->GetJoint(controlJointName);
    if (!this->controlJoint)
    {
      gzerr << "Joint with name[" << controlJointName << "] does not exist.\n";
    }
  }

}

/////////////////////////////////////////////////
void LiftDragPlugin::OnUpdate()
{
  GZ_ASSERT(this->link, "Link was NULL");
  // get linear velocity at cp in inertial frame
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel = this->link->WorldLinearVel(this->cp) - wind_vel_;
  const common::Time current_time = this->world->SimTime();
#else
  ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->cp)) - wind_vel_;
  const common::Time current_time = this->world->GetSimTime();
#endif
  ignition::math::Vector3d velI = vel;
  velI.Normalize();
  const double dt = (current_time - this->last_pub_time).Double();

  if (vel.Length() <= 0.01)
    return;

  // pose of body
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = this->link->WorldPose();
#else
  ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif

  // rotate forward and upward vectors into inertial frame
  ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);

  if (forwardI.Dot(vel) <= 0.0){
    // Only calculate lift or drag if the wind relative velocity is in the same direction
    return;
  }

  ignition::math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
    ignition::math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  // spanwiseI: a vector normal to lift-drag-plane described in inertial frame
  ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();

  const double minRatio = -1.0;
  const double maxRatio = 1.0;
  // check sweep (angle between velI and lift-drag-plane)
  double sinSweepAngle = ignition::math::clamp(
      spanwiseI.Dot(velI), minRatio, maxRatio);

  this->sweep = asin(sinSweepAngle);

  // truncate sweep to within +/-90 deg
  while (fabs(this->sweep) > 0.5 * M_PI)
    this->sweep = this->sweep > 0 ? this->sweep - M_PI
                                  : this->sweep + M_PI;
  // get cos from trig identity
  double cosSweepAngle = sqrt(1.0 - sin(this->sweep) * sin(this->sweep));

  // angle of attack is the angle between
  // velI projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
  //
  // so,
  // removing spanwise velocity from vel
  ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;

  // get direction of drag
  ignition::math::Vector3d dragDirection = -velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
  ignition::math::Vector3d liftI = spanwiseI.Cross(velInLDPlane);
  liftI.Normalize();

  // get direction of moment
  ignition::math::Vector3d momentDirection = spanwiseI;

  // compute angle between upwardI and liftI
  // in general, given vectors a and b:
  //   cos(theta) = a.Dot(b)/(a.Length()*b.Lenghth())
  // given upwardI and liftI are both unit vectors, we can drop the denominator
  //   cos(theta) = a.Dot(b)
  double cosAlpha = ignition::math::clamp(liftI.Dot(upwardI), minRatio, maxRatio);

  // Is alpha positive or negative? Test:
  // forwardI points toward zero alpha
  // if forwardI is in the same direction as lift, alpha is positive.
  // liftI is in the same direction as forwardI?
  if (liftI.Dot(forwardI) >= 0.0)
    this->alpha = this->alpha0 + acos(cosAlpha);
  else
    this->alpha = this->alpha0 - acos(cosAlpha);

  // normalize to within +/-90 deg
  while (fabs(this->alpha) > 0.5 * M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - M_PI
                                  : this->alpha + M_PI;

  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.Length();
  double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

  // get CL/CD/Cm from look up table
  double cl = this->cl_table.get(this->alpha).value_or(0);
  // In default implementation lift/momentum should always be >= 0 (should we do the same ??)
  double cd = this->cd_table.get(this->alpha).value_or(0);
  double cm = this->cm_table.get(this->alpha).value_or(0);

  double control_cl = 0, control_cd = 0, control_cm = 0;
  // modify CL/CD/Cm per control joint value
  double controlAngle;
  if (this->controlJoint)
  {
#if GAZEBO_MAJOR_VERSION >= 9
    controlAngle = this->controlJoint->Position(0);
#else
    controlAngle = this->controlJoint->GetAngle(0).Radian();
#endif
    control_cl = this->cl_flap_table.get(this->alpha).value_or(0) * controlAngle * 180.0 / M_PI;
    control_cd = this->cd_flap_table.get(this->alpha).value_or(0) * controlAngle * 180.0 / M_PI;
    control_cm = this->cm_flap_table.get(this->alpha).value_or(0) * controlAngle * 180.0 / M_PI;
  }

  // make sure drag is positive
  cd = fabs(cd);

  // compute lift force at cp
  ignition::math::Vector3d lift = ( cl * (this->area - this->control_area) + control_cl * (this->control_area) ) * q * liftI;

  // drag at cp
  ignition::math::Vector3d drag = ( cd * (this->area - this->control_area) + control_cd * (this->control_area) ) * q * dragDirection;

  // compute moment (torque) at cp
  ignition::math::Vector3d moment = ( cm * (this->area - this->control_area) + control_cm * (this->control_area) )  * q * momentDirection;

  // force about cg in inertial frame
  ignition::math::Vector3d force = lift + drag;

  // debug
  //
  // if ((this->link->GetName() == "wing_1" ||
  //      this->link->GetName() == "wing_2") &&
  //     (vel.Length() > 50.0 &&
  //      vel.Length() < 50.0))
  if (0)
  {
    gzdbg << "=============================\n";
    gzdbg << "sensor: [" << this->GetHandle() << "]\n";
    gzdbg << "Link: [" << this->link->GetName()
          << "] pose: [" << pose
          << "] dynamic pressure: [" << q << "]\n";
    gzdbg << "spd: [" << vel.Length()
          << "] vel: [" << vel << "]\n";
    gzdbg << "LD plane spd: [" << velInLDPlane.Length()
          << "] vel : [" << velInLDPlane << "]\n";
    gzdbg << "forward (inertial): " << forwardI << "\n";
    gzdbg << "upward (inertial): " << upwardI << "\n";
    gzdbg << "lift dir (inertial): " << liftI << "\n";
    gzdbg << "Span direction (normal to LD plane): " << spanwiseI << "\n";
    gzdbg << "sweep: " << this->sweep << "\n";
    gzdbg << "alpha: " << this->alpha << "\n";
    gzdbg << "lift: " << lift << "\n";
    gzdbg << "drag: " << drag << " cd: " << cd << "\n";
    gzdbg << "moment: " << moment << "\n";
    gzdbg << "force: " << force << "\n";
    gzdbg << "moment: " << moment << "\n";
  }

  // Correct for nan or inf
  force.Correct();
  this->cp.Correct();
  moment.Correct();

  // apply forces at cg (with torques for position shift)
  this->link->AddForceAtRelativePosition(force, this->cp);
  this->link->AddTorque(moment);

  auto relative_center = this->link->RelativePose().Pos() + this->cp;

  // Publish force and center of pressure for potential visual plugin.
  // - dt is used to control the rate at which the force is published
  // - it only gets published if 'topic_name' is defined in the sdf
  if (dt > 1.0 / 10 && this->sdf->HasElement("topic_name")) {
      msgs::Vector3d* force_center_msg = new msgs::Vector3d;
      force_center_msg->set_x(relative_center.X());
      force_center_msg->set_y(relative_center.Y());
      force_center_msg->set_z(relative_center.Z());

      msgs::Vector3d* force_vector_msg = new msgs::Vector3d;
      force_vector_msg->set_x(force.X());
      force_vector_msg->set_y(force.Y());
      force_vector_msg->set_z(force.Z());

      physics_msgs::msgs::Force force_msg;
      force_msg.set_allocated_center(force_center_msg);
      force_msg.set_allocated_force(force_vector_msg);

      lift_force_pub_->Publish(force_msg);
      this->last_pub_time = current_time;
  }
}

void LiftDragPlugin::WindVelocityCallback(const boost::shared_ptr<const physics_msgs::msgs::Wind> &msg) {
  wind_vel_ = ignition::math::Vector3d(msg->velocity().x(),
            msg->velocity().y(),
            msg->velocity().z());
}
