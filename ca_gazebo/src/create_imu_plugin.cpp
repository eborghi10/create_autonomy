/*
 * Copyright 2013 Open Source Robotics Foundation
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
/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 */

#include <string>

#include <ca_gazebo/create_imu_plugin.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosCreateIMU)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosCreateIMU::GazeboRosCreateIMU()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosCreateIMU::~GazeboRosCreateIMU()
{
  this->update_connection_.reset();
  // Finalize the controller
  this->gazebo_ros_->node()->shutdown();
  this->callback_queue_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCreateIMU::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // save pointers
  this->model_ = _parent;
  this->world_ = this->model_->GetWorld();
  this->sdf_ = _sdf;

  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosCreateIMU::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCreateIMU::LoadThread()
{
  this->gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( this->model_, this->sdf_, "Imu" ) );
  // Make sure the ROS node for Gazebo has already been initialized
  this->gazebo_ros_->isInitialized();

  this->gazebo_ros_->getParameter<std::string> (this->service_name_, "serviceName", "/default_imu");
  this->gazebo_ros_->getParameter<std::string> (this->topic_name_, "topicName", "/default_imu");
  this->gazebo_ros_->getParameter<double> (this->gaussian_noise_, "gaussianNoise", 0.0);
  this->gazebo_ros_->getParameter<std::string> (this->link_name_ , "bodyName", "");
  ignition::math::Vector3d xyzOffset, rpyOffset;
  this->gazebo_ros_->getParameter<ignition::math::Vector3d>
      (xyzOffset, "xyzOffset", ignition::math::Vector3d(0, 0, 0));
  this->offset_.Pos() = xyzOffset;
  this->gazebo_ros_->getParameter<ignition::math::Vector3d>
      (rpyOffset, "rpyOffset", ignition::math::Vector3d(0, 0, 0));
  this->offset_.Rot() = ignition::math::Quaterniond(rpyOffset);
  this->gazebo_ros_->getParameter<double> (this->update_rate_, "updateRate", 0.0);
  std::string frame;
  this->gazebo_ros_->getParameter<std::string> (frame, "frameName", "");
  this->frame_name_ = this->gazebo_ros_->resolveTF(frame);

  // publish multi queue
  this->pmq.startServiceThread();

  // assert that the body by link_name_ exists
  this->link = boost::dynamic_pointer_cast<physics::Link>(
#if GAZEBO_MAJOR_VERSION >= 8
    this->world_->EntityByName(this->link_name_));
#else
    this->world_->GetEntity(this->link_name_));
#endif
  if (!this->link)
  {
    ROS_FATAL_NAMED("imu", "gazebo_ros_imu plugin error: bodyName: %s does not exist\n",
      this->link_name_.c_str());
    return;
  }

  // if topic name specified as empty, do not publish
  if (this->topic_name_ != "")
  {
    this->pub_Queue = this->pmq.addPub<sensor_msgs::Imu>();
    this->pub_ = this->gazebo_ros_->node()->advertise<sensor_msgs::Imu>(
      this->topic_name_, 1);

    // advertise services on the custom queue
    ros::AdvertiseServiceOptions aso =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      this->service_name_, boost::bind(&GazeboRosCreateIMU::ServiceCallback,
      this, _1, _2), ros::VoidPtr(), &this->imu_queue_);
    this->srv_ = this->gazebo_ros_->node()->advertiseService(aso);
  }

  // Initialize the controller
#if GAZEBO_MAJOR_VERSION >= 8
  this->last_time_ = this->world_->SimTime();
#else
  this->last_time_ = this->world_->GetSimTime();
#endif

  // this->initial_pose_ = this->link->GetPose();
#if GAZEBO_MAJOR_VERSION >= 8
  this->last_vpos_ = this->link->WorldLinearVel();
  this->last_veul_ = this->link->WorldAngularVel();
#else
  this->last_vpos_ = this->link->GetWorldLinearVel().Ign();
  this->last_veul_ = this->link->GetWorldAngularVel().Ign();
#endif
  this->apos_ = 0;
  this->aeul_ = 0;

  // start custom queue for imu
  this->callback_queue_thread_ =
    boost::thread(boost::bind(&GazeboRosCreateIMU::IMUQueueThread, this));


  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosCreateIMU::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// returns true always, imu is always calibrated in sim
bool GazeboRosCreateIMU::ServiceCallback(const std_srvs::Empty::Request &req,
                                         const std_srvs::Empty::Response &res)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosCreateIMU::UpdateChild()
{
#if GAZEBO_MAJOR_VERSION >= 8
  common::Time cur_time = this->world_->SimTime();
#else
  common::Time cur_time = this->world_->GetSimTime();
#endif

  // rate control
  if (this->update_rate_ > 0 &&
      (cur_time - this->last_time_).Double() < (1.0 / this->update_rate_))
    return;

  if ((this->pub_.getNumSubscribers() > 0 && this->topic_name_ != ""))
  {
    ignition::math::Pose3d pose;
    ignition::math::Quaterniond rot;
    ignition::math::Vector3d pos;

    // Get Pose/Orientation ///@todo: verify correctness
#if GAZEBO_MAJOR_VERSION >= 8
    pose = this->link->WorldPose();
#else
    pose = this->link->GetWorldPose().Ign();
#endif
    // apply xyz offsets and get position and rotation components
    pos = pose.Pos() + this->offset_.Pos();
    rot = pose.Rot();

    // apply rpy offsets
    rot = this->offset_.Rot()*rot;
    rot.Normalize();

    // get Rates
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d vpos = this->link->WorldLinearVel();
    ignition::math::Vector3d veul = this->link->WorldAngularVel();
#else
    ignition::math::Vector3d vpos = this->link->GetWorldLinearVel().Ign();
    ignition::math::Vector3d veul = this->link->GetWorldAngularVel().Ign();
#endif

    // differentiate to get accelerations
    double tmp_dt = this->last_time_.Double() - cur_time.Double();
    if (tmp_dt != 0)
    {
      this->apos_ = (this->last_vpos_ - vpos) / tmp_dt;
      this->aeul_ = (this->last_veul_ - veul) / tmp_dt;
      this->last_vpos_ = vpos;
      this->last_veul_ = veul;
    }

    // copy data into pose message
    this->imu_msg_.header.frame_id = this->frame_name_;
    this->imu_msg_.header.stamp.sec = cur_time.sec;
    this->imu_msg_.header.stamp.nsec = cur_time.nsec;

    // orientation quaternion

    // uncomment this if we are reporting orientation in the local frame
    // not the case for our imu definition
    // // apply fixed orientation offsets of initial pose
    // rot = this->initial_pose_.Rot()*rot;
    // rot.Normalize();

    this->imu_msg_.orientation.x = rot.X();
    this->imu_msg_.orientation.y = rot.Y();
    this->imu_msg_.orientation.z = rot.Z();
    this->imu_msg_.orientation.w = rot.W();

    // pass euler angular rates
    ignition::math::Vector3d linear_velocity(
      veul.X() + this->GaussianKernel(0, this->gaussian_noise_),
      veul.Y() + this->GaussianKernel(0, this->gaussian_noise_),
      veul.Z() + this->GaussianKernel(0, this->gaussian_noise_));
    // rotate into local frame
    // @todo: deal with offsets!
    linear_velocity = rot.RotateVector(linear_velocity);
    this->imu_msg_.angular_velocity.x    = linear_velocity.X();
    this->imu_msg_.angular_velocity.y    = linear_velocity.Y();
    this->imu_msg_.angular_velocity.z    = linear_velocity.Z();

    // pass accelerations
    ignition::math::Vector3d linear_acceleration(
      apos_.X() + this->GaussianKernel(0, this->gaussian_noise_),
      apos_.Y() + this->GaussianKernel(0, this->gaussian_noise_),
      apos_.Z() + this->GaussianKernel(0, this->gaussian_noise_));
    // rotate into local frame
    // @todo: deal with offsets!
    linear_acceleration = rot.RotateVector(linear_acceleration);
    this->imu_msg_.linear_acceleration.x    = linear_acceleration.X();
    this->imu_msg_.linear_acceleration.y    = linear_acceleration.Y();
    this->imu_msg_.linear_acceleration.z    = linear_acceleration.Z();

    // fill in covariance matrix
    /// @todo: let user set separate linear and angular covariance values.
    /// @todo: apply appropriate rotations from frame_pose
    double gn2 = this->gaussian_noise_*this->gaussian_noise_;
    this->imu_msg_.orientation_covariance[0] = gn2;
    this->imu_msg_.orientation_covariance[4] = gn2;
    this->imu_msg_.orientation_covariance[8] = gn2;
    this->imu_msg_.angular_velocity_covariance[0] = gn2;
    this->imu_msg_.angular_velocity_covariance[4] = gn2;
    this->imu_msg_.angular_velocity_covariance[8] = gn2;
    this->imu_msg_.linear_acceleration_covariance[0] = gn2;
    this->imu_msg_.linear_acceleration_covariance[4] = gn2;
    this->imu_msg_.linear_acceleration_covariance[8] = gn2;

    {
      boost::mutex::scoped_lock lock(this->lock_);
      // publish to ros
      if (this->pub_.getNumSubscribers() > 0 && this->topic_name_ != "")
          this->pub_Queue->push(this->imu_msg_, this->pub_);
    }

    // save last time stamp
    this->last_time_ = cur_time;
  }
}


//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosCreateIMU::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosCreateIMU::IMUQueueThread()
{
  static const double timeout = 0.01;

  while (gazebo_ros_->node()->ok())
  {
    this->imu_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}  // namespace gazebo