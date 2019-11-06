#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

static const ros::Duration update_rate = ros::Duration(1); // 1 Hz
namespace gazebo
{
class ModelPosePublisherPlugin : public ModelPlugin
{

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Make sure the ROS node for Gazebo has already been initialized
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }
      ROS_INFO("Model pose publisher started!");
      ROS_INFO("model Name = %s", _parent->GetName().c_str());

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPosePublisherPlugin::OnUpdate, this));
      this->prev_update_time_ = ros::Time::now();

      this->rosnode_ = new ros::NodeHandle("ModelPose");
      this->pub_ = this->rosnode_->advertise<geometry_msgs::Pose>("/ca_gazebo/model_pose", 100);
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      if ((ros::Time::now() - this->prev_update_time_) < update_rate) {
       return;
      }

      geometry_msgs::Pose msg;
      msg.position.x = this->model->GetLink("link")->WorldPose().Pos().X();
      msg.position.y = this->model->GetLink("link")->WorldPose().Pos().Y();
      msg.position.z = this->model->GetLink("link")->WorldPose().Pos().Z();

      msg.orientation.x = this->model->GetLink("link")->WorldPose().Rot().X();
      msg.orientation.y = this->model->GetLink("link")->WorldPose().Rot().Y();
      msg.orientation.z = this->model->GetLink("link")->WorldPose().Rot().Z();
      msg.orientation.w = this->model->GetLink("link")->WorldPose().Rot().W();

      this->pub_.publish(msg);

      this->prev_update_time_ = ros::Time::now();
    }

    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    // Pointer to the model
    private: physics::ModelPtr model;

    private: ros::Time prev_update_time_;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
   };

   // Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(ModelPosePublisherPlugin)
}
