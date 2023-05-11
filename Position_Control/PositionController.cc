/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
 
 * Note:
 Pose information will be sent in meters and degrees. After that the program convert to radians and will ONLY work in radians

 */

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>
#include <tuple>
#include <utility>
#include <limits>
#include <chrono>
#include <iostream>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <fstream>


#include <gz/msgs/twist.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Util.hh"

#include "PositionController.hh"

# define M_PIl          3.141592653589793238462643383279502884L /* pi */

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::PositionControllerPrivate
{
  // ------------------------- Funtion Declarations ------------------------
  /// \brief Callback for wrench subscription
  /// \param[in] _msg String message (Containing Pose Message)
  public: void OnPosePub(const msgs::StringMsg &_msg);

  /// \brief Callback for file with wrench info subscription
  /// \param[in] _msg String message (Containing File name)
  public: void OnFilePub(const msgs::StringMsg &_msg);

  /// \brief Loads the provided file (with pose instructions). Returns true if file loaded successfully 
  /// \param[in] filename String containing the filename
  public: bool FileLoad(const std::string filename);

  public: bool ErrorCheckPoseMsg(const std::string message);

  public: void UpdateFileLine();

  /// \brief Updates the pose and time target. Returns true if targets updated successfully
  public: void UpdateTargets(const std::string message);

  /// \brief Calculates the time goals and properties after the Model Pose Targets has been updates
  public: void UpdateTime(const gz::sim::UpdateInfo &_info);

  /// \brief Calculates the required force to move model to desired pose and updates the force and torque values
  /// \param[in] _msg String message (Containing Pose message)
  public: void UpdateWrench(const gz::sim::UpdateInfo &_info, const EntityComponentManager &_ecm, double movement_time);

  /// \brief Returns the force required to move an object from its current to a new position within a set amount of time
  /// \param[in] current_position Single coordinate of current position (in meters)
  /// \param[in] next_position Single coordinate of position object has to move to (in meters)
  /// \param[in] mass Mass of the object (in kg)
  /// \param[in] movement_time Time in which object has to reach the 'next_position' (in seconds)
  /// \param[in] timestep_size Time over which force will be applied to the object (= timestep size of the simulator) (in seconds)
  private: double CalcForce(double current_position, double next_position, double mass, double movement_time, double timestep_size);

  /// \brief Calculates the Rotation matrix from Euler Angles, in the order ZYX
  /// \param[in] roll Angle of roll about x-axis (in radians)
  /// \param[in] pitch Angle of pitch about y-axis (in radians)
  /// \param[in] yaw Angle of yaw about z-axis (in radians)
  Eigen::Matrix3d CalcRotationMatrix(double roll, double pitch, double yaw);

  /// \brief Returns the torque (wrench) required to rotate an object from its current to a new orientation within a set amount of time
  /// \param[in] current_orientation Single-axis angle of current orientation (in degrees)
  /// \param[in] next_orientation Single-axis orientation object has to rotate to (in degrees)
  /// \param[in] mass_moment_of_inertia Mass moment of inertia of the object (in XX)
  /// \param[in] movement_time Time in which object has to reach the 'next_orientation' (in seconds)
  /// \param[in] timestep_size Time over which force will be applied to the object (= timestep size of the simulator) (in seconds)
  private: double CalcTorque(double orientation, double mass_moment_of_inertia, double movement_time, double timestep_size);

  /// \brief Returns the opposite vector of the one provided (changes direction)
  /// \param[in] vector Vector which direction should be changed
  public: math::Vector3d FlipVector3D(math::Vector3d vector);


  // -------------------------       Variables      ------------------------
  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief The model's canonical link.
  public: Link canonicalLink{kNullEntity};
    
  /// \brief Stores the Wrench and Force to apply to the model (once applied, it stores the values for later reference)
  public: math::Vector3d wrench_force = {0, 0, 0};
  public: math::Vector3d wrench_torque = {0, 0, 0};

  /// \brief Allow specifying constant xyz and rpy offsets
  public: gz::math::Pose3d pose_offset = {0, 0, 0, 0, 0, 0};

  /// \brief Specifies the xyz and rpy pose targets
  public: gz::math::Pose3d pose_target = {0, 0, 0, 0, 0, 0};

  /// \brief Specifies the time in which to reach the pose target
  public: double time_target;

  /// \brief Specifies the time at which the pose target should be reached
  public: std::chrono::duration<double> time_destination;

  /// \brief Specifies the size of the timestep
  public: double time_step_size;

  /// \brief Specifies the time when the precision timestep has been reached
  public: std::chrono::duration<double> time_precision_destination;

  /// \brief Shows if pose message received on topic, true until destination reached or overwritten by a new topic message (pose or file)
  public: bool on_topic_pose = false;

  /// \brief  Shows if meesage of file topic received, true until file finished or overwritten by a new topic message (pose or file)
  public: bool on_topic_file = false;

  /// \brief Shows if Wrench should be applied during pre-update. (True if it should be applied)
  public: bool apply_wrench = false;

  /// \brief fstream constaining the pose data_file
  public: std::fstream data_file;

  // -------------------------------------- Publisher Nodes
  /// \brief Gazebo communication node.
  public: transport::Node node;
};

//////////////////////////////////////////////////
PositionController::PositionController()
    : dataPtr(std::make_unique<PositionControllerPrivate>())
{
}

//////////////////////////////////////////////////
void PositionController::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "Position Controller plugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  // this->dataPtr->model
  //_ecm.Component<components::Mass>(this->dataPtr->model);

  // double mass = this->dataPtr->model.GetInertial().GetMass();
  //std::string name = this->dataPtr->model.Name(_ecm);
  //this->dataPtr->model.GetLink(_ecm);

  // Print the mass to the console
  // gzmsg << "Model mass: " << mass << std::endl
  // ---------------------- Settings from the SDF file.
  // -Necessary
  
  // -Optional
  // ---------Subscribe to Pose Topic
  std::vector<std::string> modelTopic_pose_subscriber;
  if (_sdf->HasElement("pose_topic"))
  {
    modelTopic_pose_subscriber.push_back(_sdf->Get<std::string>("pose_topic"));
  }
  else {
    modelTopic_pose_subscriber.push_back(
    "/model/" + this->dataPtr->model.Name(_ecm) + "/pos_contr");
  }
  
  auto modelT_pose_subscriber = validTopic(modelTopic_pose_subscriber);
  this->dataPtr->node.Subscribe(
      modelT_pose_subscriber, &PositionControllerPrivate::OnPosePub, this->dataPtr.get());
    gzmsg << "PositionController subscribing to String Messages (for single Pose msg) on Topic :["
        << modelT_pose_subscriber << "]"
        << std::endl;

  // ---------Subscribe to File Topic
  std::vector<std::string> modelTopic_file_subscriber;
  if (_sdf->HasElement("file_topic"))
  {
    modelTopic_file_subscriber.push_back(_sdf->Get<std::string>("file_topic"));
  }
  else {
    modelTopic_file_subscriber.push_back(
    "/model/" + this->dataPtr->model.Name(_ecm) + "/file_pos_contr");
  }
  

  auto modelT_file_subscriber = validTopic(modelTopic_file_subscriber);
  this->dataPtr->node.Subscribe(
      modelT_file_subscriber, &PositionControllerPrivate::OnFilePub, this->dataPtr.get());
    gzmsg << "PositionController subscribing to String Messages (for Pose file) on Topic :["
        << modelT_file_subscriber << "]"
        << std::endl;


  if (_sdf->HasElement("xyz_offset"))
  {
    this->dataPtr->pose_offset.Pos() = _sdf->Get<gz::math::Vector3d>(
      "xyz_offset");
  }

  if (_sdf->HasElement("rpy_offset"))
  {
    this->dataPtr->pose_offset.Rot() =
      gz::math::Quaterniond(_sdf->Get<gz::math::Vector3d>(
        "rpy_offset"));
  }

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), components::CanonicalLink());
  if (!links.empty()) {
    this->dataPtr->canonicalLink = Link(links[0]);
  }


}

//////////////////////////////////////////////////
void PositionController::PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("PositionController::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // New force info has been given
  if (this->dataPtr->apply_wrench)
  {
    this->dataPtr->apply_wrench = false;
    // Stop model (apply opposite forces)
    this->dataPtr->canonicalLink.AddWorldWrench(_ecm, this->dataPtr->FlipVector3D(this->dataPtr->wrench_force), this->dataPtr->FlipVector3D(this->dataPtr->wrench_torque));
    this->dataPtr->wrench_force.Set(0,0,0); this->dataPtr->wrench_torque.Set(0,0,0); // Reset Force Amounts
    // Calculate new forces
    this->dataPtr->UpdateTime(_info);
    this->dataPtr->UpdateWrench(_info, _ecm, this->dataPtr->time_target);
    this->dataPtr->canonicalLink.AddWorldWrench(_ecm, this->dataPtr->wrench_force, this->dataPtr->wrench_torque); // Apply new forces
  }

  std::chrono::duration<double> ElapsedTime = _info.simTime;

  // Time for which force should be applied has been reached
  if (this->dataPtr->time_destination.count() == ElapsedTime.count())
  {
    //gzmsg << "At the final time step:  " << ElapsedTime.count() << std::endl;
    this->dataPtr->canonicalLink.AddWorldWrench(_ecm, this->dataPtr->FlipVector3D(this->dataPtr->wrench_force), this->dataPtr->FlipVector3D(this->dataPtr->wrench_torque));

    this->dataPtr->wrench_force.Set(0,0,0); this->dataPtr->wrench_torque.Set(0,0,0); // Reset Force Amounts

    // Currently reading pose from a file
    if (this->dataPtr->on_topic_file)
    {
      this->dataPtr->UpdateFileLine();
    }

  }

}

//////////////////////////////////////////////////
void PositionController::PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm)
{
  GZ_PROFILE("PositionController::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;


}

//////////////////////////////////////////////////
void PositionControllerPrivate::OnPosePub(const msgs::StringMsg &_msg)
{
  // gzmsg << "Pose msg Received" << std::endl;
  //std::lock_guard<std::mutex> lock(this->mutex);
  //this->targetVel = _msg;
  if (PositionControllerPrivate::ErrorCheckPoseMsg(_msg.data())) {
    this->on_topic_pose = true; this->on_topic_file = false;
    PositionControllerPrivate::UpdateTargets(_msg.data());
  }
}

void PositionControllerPrivate::OnFilePub(const msgs::StringMsg &_msg)
{
  //std::lock_guard<std::mutex> lock(this->mutex);
  //this->targetVel = _msg;
  if (PositionControllerPrivate::FileLoad(_msg.data())) {
    this->on_topic_file = true; this->on_topic_pose = false;
    PositionControllerPrivate::UpdateFileLine();
  }
}

bool PositionControllerPrivate::FileLoad(const std::string filename)
{
  gzmsg << filename << std::endl;
  std::fstream file;
  file.open(filename, std::ios::in);
  if (file.fail())
  {
    gzerr << "File: \'" << filename << "\' not found" << std::endl;
    return false;
  }
  else 
  {
    /* 
     \ TODO
    Error Check the file (make sure all lines are formatted correctly)
      */

    bool error = false;
    std::string line;
    int line_error_found = 0;
    while (std::getline(file, line))
    {
     line_error_found ++;
     if(!PositionControllerPrivate::ErrorCheckPoseMsg(line))
     {
      gzerr << "Error found in file, line: " << line_error_found;
      return false;
     }
    }
     
    // No errors found
    file.close();
    this->data_file.open(filename, std::ios::in);
    return true;  
  }
}


bool PositionControllerPrivate::ErrorCheckPoseMsg(const std::string message)
{
  std::vector<float> msg_vector;
  std::stringstream s_stream(message); //Create string stream from the string
   while(s_stream.good()) {
      std::string substr;
      getline(s_stream, substr, ','); //Get first string delimited by comma
      try {
        msg_vector.push_back(std::stof(substr)); // Push string to msg_result and change to float
      }
      catch (const std::invalid_argument& e){
        gzerr << "Invalid argument type given (must be of type float)" << std::endl;
        return false;
      }
      
   }   
   if (msg_vector.size() != 7){
    gzerr << "Wrong number of variables have been provided" << std::endl;
    return false;
   }

  if ( (msg_vector[4] < -90) || (msg_vector[4] > 90) ) {
    gzerr << "Pitch value exceeds limits. Must be in range: -90 deg < Pitch < 90 deg" << std::endl;
    return false;
  }

  return true;
}

void PositionControllerPrivate::UpdateFileLine()
{
      // Read next line
      std::string line;   
      if (std::getline(this->data_file, line)) // Not end of file -> Update forces
      {
        this->apply_wrench = true;
        this->UpdateTargets(line);
      }
      else // End of file, stop 'on_topic_file', close file
      {
        this->on_topic_file = false;
        this->data_file.close();
      }
}

// Updates the force and torque values to be applied to the model.
// If message format is wrong -> model force will not be updated
void PositionControllerPrivate::UpdateTargets(const std::string message)
{
  std::vector<float> msg_vector;
  std::stringstream s_stream(message); //Create string stream from the string
   while(s_stream.good()) {
      std::string substr;
      getline(s_stream, substr, ','); //Get first string delimited by comma
      msg_vector.push_back(std::stof(substr)); // Push string to msg_result and change to float
    }

  // Degrees converted to radians
  this->pose_target.Set(msg_vector[0], msg_vector[1], msg_vector[2], msg_vector[3]*(M_PIl/180), msg_vector[4]*(M_PIl/180), msg_vector[5]*(M_PIl/180));
  this->time_target = msg_vector[6];
  this->apply_wrench = true;
}

void PositionControllerPrivate::UpdateTime(const gz::sim::UpdateInfo &_info)
{
  // Calculate time when destination has to be reached
  using fsec = std::chrono::duration<float>;
  auto time_target_chrono = std::chrono::round<std::chrono::nanoseconds>(fsec{this->time_target});
  
  this->time_destination = _info.simTime + time_target_chrono;

  // Get Time step size (Note that duration has to be parsed to a double)
  std::chrono::duration<double> step_size = _info.dt;
  this->time_step_size = step_size.count(); 
}

void PositionControllerPrivate::UpdateWrench(const gz::sim::UpdateInfo &_info, const EntityComponentManager &_ecm, double movement_time)
{
  // - Find Current Pose of Model
  // Create the pose component if it does not exist.
  auto pos = _ecm.Component<components::Pose>(this->model.Entity());
  // if (!pos)
  // {
  //   _ecm.CreateComponent(this->model.Entity(), components::Pose());
    // }

  // Get and set robotBaseFrame to odom transformation.

  const math::Pose3d rawPose = worldPose(this->model.Entity(), _ecm);
  math::Pose3d pose = rawPose * this->pose_offset;


  /* -----------------
    \TODO
    Find mass and mass moment of inertia from model entity (SDF file)
  --------------- */
  double mass = 1.0; double Ix = 1; double Iy = 1; double Iz = 1;

  
  // -- Calculate Forces (Remove any force that might still be applied to object)
  double force_x = CalcForce(pose.Pos().X(), this->pose_target.X(), mass, movement_time, this->time_step_size); 
  double force_y = CalcForce(pose.Pos().Y(), this->pose_target.Y(), mass, movement_time, this->time_step_size);
  double force_z = CalcForce(pose.Pos().Z(), this->pose_target.Z(), mass, movement_time, this->time_step_size);  
  this->wrench_force.Set(force_x, force_y, force_z);

  // -- Calculate Torques
  double delta_roll = this->pose_target.Rot().Roll() - pose.Rot().Roll();
  double delta_pitch = this->pose_target.Rot().Pitch() - pose.Rot().Pitch();
  double delta_yaw = this->pose_target.Rot().Yaw() - pose.Rot().Yaw();

  // Calculate current position Rotation Matrix
  Eigen::Matrix3d r_current;
  r_current = CalcRotationMatrix(pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw());
  
  // Calculate next position Rotation Matrix
  Eigen::Matrix3d r_next;
  r_next = CalcRotationMatrix(pose_target.Rot().Roll(), pose_target.Rot().Pitch(), pose_target.Rot().Yaw());
  
  // Calculate Difference Rotation matrix
  Eigen::Matrix3d R_diff;
  R_diff = r_next*(r_current.transpose());

  // Define Variables
  Eigen::Vector3d torque {0,0,0};
  double angle = 0;
  Eigen::Vector3d axis {0,0,0}; // Axis of 'Axis angle representation'
  Eigen::Vector3d axis_components {0,0,0}; // Components of axis vector along XYZ axis, using the angle (shows actual rotation around each axis is global reference frame)

  // Convert difference matrix to Axis-Angle representation
  angle = acos((R_diff.trace() - 1) / 2);
  
  if ( (angle > 0.001) || (angle < -0.001) ) //If Angle is large enough, object actually rotates and new torques have to be calculated (0.001 rad  = 0.057 deg)
  {
    Eigen::Vector3d axis {R_diff(2,1) - R_diff(1,2), R_diff(0,2) - R_diff(2,0), R_diff(1,0) - R_diff(0,1)};
    axis /= (2 * sin(angle));
    axis = axis.normalized();
    // Find XYZ angle components from axis-angle representation
    axis_components(0) = axis(0) * angle;
    axis_components(1) = axis(1) * angle;
    axis_components(2) = axis(2) * angle;

    // Calculate Torques
    torque(0) = CalcTorque(axis_components(0), Ix, movement_time, this->time_step_size);
    torque(1) = CalcTorque(axis_components(1), Iy, movement_time, this->time_step_size);
    torque(2) = CalcTorque(axis_components(2), Iz, movement_time, this->time_step_size);
  } 

  //this->dataPtr->wrench_torque.Set(0,0,0); // Reset Force Amounts
  this->wrench_torque.Set(torque(0), torque(1), torque(2));

  // Messages (uncomment as needed)
  gzmsg << "Movement time is: " << movement_time << std::endl;
  gzmsg << "Current Position: x-> " << pose.Pos().X() << " y-> " << pose.Y() << " z-> " << pose.Z() << std::endl; 
  gzmsg << "Target Potition: x-> " << this->pose_target.X() << " y-> " << this->pose_target.Y() << " z-> " << this->pose_target.Z() << std::endl; 
  gzmsg << "Forces to apply: x-> " << force_x << " y-> " << force_y << " z-> " << force_z << std::endl;

  gzmsg << "Current Angles: r-> " << pose.Rot().Roll() << " p-> " << pose.Rot().Pitch() << " y-> " << pose.Rot().Yaw() << std::endl; 
  gzmsg << "Target Angles: r-> " << this->pose_target.Rot().Roll() << " p-> " << this->pose_target.Rot().Pitch() << " y-> " << this->pose_target.Rot().Yaw() << std::endl; 
  gzmsg << "Change in Angles: r-> " << delta_roll << " p-> " << delta_pitch << " y-> " << delta_yaw << std::endl; 

  gzmsg << "R_Cur: " << std::endl << r_current <<std::endl;
  gzmsg << "R_Next: " << std::endl << r_next <<std::endl;
  gzmsg << "R_Diff: " << std::endl << R_diff <<std::endl;

  gzmsg << "Angle: " << angle << std::endl;
  gzmsg << "Axis: " << axis.transpose() << std::endl;
  gzmsg << "Change in rpy according to axis  r: " << axis_components(0) << " p: " << axis_components(1) <<  " y: " << axis_components(2) << std::endl;
  gzmsg << "Torques to Apply  x: " << torque(0) << " y: " << torque(1) <<  " z: " << torque(2) << std::endl;
}

double PositionControllerPrivate::CalcForce(double current_position, double next_position, double mass, double movement_time, double timestep_size)
{
  double distance = next_position - current_position;
  double velocity = distance/movement_time;
  return mass*velocity/timestep_size;
}

Eigen::Matrix3d PositionControllerPrivate::CalcRotationMatrix(double roll, double pitch, double yaw)
{
  Eigen::Matrix3d rx { {1,0,0}, {0, std::cos(roll), -1*std::sin(roll)}, {0, std::sin(roll), std::cos(roll)}};
  Eigen::Matrix3d ry { {std::cos(pitch),0,std::sin(pitch)}, {0,1,0}, {-1*std::sin(pitch),0,std::cos(pitch)}};
  Eigen::Matrix3d rz { {std::cos(yaw),-1*std::sin(yaw),0}, {std::sin(yaw),std::cos(yaw),0}, {0,0,1}};
  return rz * (ry * rx);
}

double PositionControllerPrivate::CalcTorque(double angle, double mass_moment_of_inertia, double movement_time, double timestep_size)
{
  double angular_velocity = angle/movement_time;
  double torque = mass_moment_of_inertia*angular_velocity/time_step_size;
  if (std::abs(torque) < 0.0001)
    torque = 0;
  return torque;
}

math::Vector3d PositionControllerPrivate::FlipVector3D(math::Vector3d vector)
{
  math::Vector3d reversed = {-1*vector.X(), -1*vector.Y(), -1*vector.Z()};
  return reversed;
}
//////////////////////////////////////////////////

GZ_ADD_PLUGIN(PositionController,
              gz::sim::System,
              PositionController::ISystemConfigure,
              PositionController::ISystemPreUpdate,
              PositionController::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(PositionController,
                    "gz::sim::systems::PositionController")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(PositionController,
                    "ignition::gazebo::systems::PositionController")
