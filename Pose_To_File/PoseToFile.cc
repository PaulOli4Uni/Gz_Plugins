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
 * gz topic -t "/model/box/cmd_vel" -m gz.msgs.Twist -p "linear: {x: -0.0}, angular: {z: -0.0}"

 */

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <chrono>
#include <iostream>
#include <cmath>

#include <gz/msgs/twist.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/header.pb.h>

#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Pose.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"


#include "PoseToFile.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::PoseToFilePrivate
{


/// \brief Callback for the video recorder service
  public: bool OnRecordPose(const msgs::VideoRecord &_msg, msgs::Int32 &_res);

/// \brief Callback for the video recorder service
  public: std::string PoseMessage(const gz::sim::UpdateInfo &_info, const EntityComponentManager &_ecm);

  public: void CheckUpdatePeriod(const gz::sim::UpdateInfo &_info);

// -------------------------       Variables      ------------------------

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Pointer to the event manager
  public: EventManager *eventMgr = nullptr;

  /// \brief Transport node
  public: transport::Node node;

  /// \brief Allow specifying constant xyz and rpy offsets
  public: gz::math::Pose3d pose_offset = {0, 0, 0, 0, 0, 0};

  /// \brief Name of service for recording video
  public: std::string service;

  /// \brief Name of file to store pose info
  public: std::string file_name;

  /// \brief Name of file to store pose info
  public: std::fstream file;

  /// \brief True when data is being collected (changed in pre/post update due to a service call)
  public: bool data_collecting;

  /// \brief True when data collecting should start (from service call)
  public: bool data_start_collect;

  /// \brief True when data collecting should finish (from service call)
  public: bool data_stop_collecting;

  /// \brief Time period between pose updates (interval 0 < update_period) [seconds]
  public: double update_period = 1.0;

  /// \brief Sim time when previous pose message was sent
  public: std::chrono::nanoseconds sim_time_at_previous_pose_update;

};
//////////////////////////////////////////////////
PoseToFile::PoseToFile()
    : dataPtr(std::make_unique<PoseToFilePrivate>())
{
}

//////////////////////////////////////////////////
void PoseToFile::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager & _eventMgr)
{
  this->dataPtr->model = Model(_entity);
  this->dataPtr->eventMgr = &_eventMgr;

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "Position Controller plugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

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

  // video recorder service topic name
  if (_sdf->HasElement("service"))
  {
    this->dataPtr->service = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("service"));

    if (this->dataPtr->service.empty())
    {
      gzerr << "Service [" << _sdf->Get<std::string>("service")
             << "] not valid. Ignoring." << std::endl;
             return;
    }
    else 
    {
      this->dataPtr->node.Advertise(this->dataPtr->service,
       &PoseToFilePrivate::OnRecordPose, this->dataPtr.get());
      gzmsg << "Record pose service on [" << this->dataPtr->service << "]" << std::endl;
    }
  }
  
  
  


}

//////////////////////////////////////////////////
void PoseToFile::PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("PoseToFile::PreUpdate");

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

  


  if (this->dataPtr->data_start_collect) // Start collecting data
  {
    // Opens file in append and truncating mode (Clears all content if any is present, and then start appending new info)
    this->dataPtr->file.open(this->dataPtr->file_name, std::ios::trunc | std::ios::app);
    //this->dataPtr->file.close();
    
    this->dataPtr->data_collecting = true;// Start data_collect bool
    
    gzmsg << " Started collecting data. Update period of: " << this->dataPtr->update_period << std::endl;
    this->dataPtr->data_start_collect = false;
  }

  // Calc update interval -> On topic receive and at configure

  // Data collecting before start collect (as there should be a time between )
  if (this->dataPtr->data_collecting)
  {
    if (true) // Check if next update time reached
    {
      
      //this->dataPtr->file.open(this->dataPtr->file_name, std::ios::app);
      
      std::string text = this->dataPtr->PoseMessage(_info, _ecm);
      this->dataPtr->file >> text;
      gzmsg << text << std::endl;
      //this->dataPtr->file.close();
    }
  }
  
}

//////////////////////////////////////////////////
void PoseToFile::PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm)
{
  GZ_PROFILE("PoseToFile::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;
  
  if (this->dataPtr->data_stop_collecting)
  {
    this->dataPtr->data_collecting = false;//Stop data collect bool
    this->dataPtr->file << this->dataPtr->PoseMessage(_info, _ecm); // Add last pose info
    this->dataPtr->file.close();// Close file

    //this->dataPtr->
    gzmsg << " Stopped collecting data. Stored in file: " << this->dataPtr->file_name << std::endl;
    this->dataPtr->data_stop_collecting = false;
  }

}

//////////////////////////////////////////////////

bool PoseToFilePrivate::OnRecordPose(const msgs::VideoRecord &_msg, msgs::Int32 &_res)
{
  std::string file_name = _msg.save_filename();

  // Get the extension of the file
  std::string extension = file_name.substr(file_name.find_last_of(".") + 1);
    
    // Convert the extension to lowercase
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower); 
    // Check if the extension is correct
    if (extension != "txt")
    {
      gzerr << "Error: incorrect file extension [Must be of type .txt]" << std::endl;
      _res.set_data(2);
      return true;
    }
    else //File has correct extention
    {
      this->file_name = file_name;
    }

  // Data Start/Stop Collecting always start as a false
  this->data_start_collect = false; this->data_stop_collecting = false;

  if (_msg.start())
    this->data_start_collect = true;

  if (_msg.stop())
    this->data_stop_collecting = true;

  if (this->data_start_collect && this->data_stop_collecting)
  {
    _res.set_data(3);
    gzerr << "Data cannot be started and stopped at same time" << std::endl;
    return true; 
  }
  else if (!this->data_start_collect && !this->data_stop_collecting)
  {
    _res.set_data(4);
    gzerr << "Data collection has to be started or stopped" << std::endl;
    return true; 
  }
  
  gzmsg << "Following data received on service: " << this->service << std::endl <<
      "Filename: " << this->file_name << " Start Collecting Data: " << 
      this->data_start_collect << " Stop Collecting Data: " << this->data_stop_collecting << std::endl;
  
  _res.set_data(1);
  return true;
}

std::string PoseToFilePrivate::PoseMessage(const gz::sim::UpdateInfo &_info, const EntityComponentManager &_ecm)
{
  auto pos = _ecm.Component<components::Pose>(this->model.Entity());
  const math::Pose3d rawPose = worldPose(this->model.Entity(), _ecm);
  math::Pose3d pose = rawPose * this->pose_offset;
  std::string pose_msg = std::to_string(pose.Pos().X()) + "," +
      std::to_string(pose.Pos().Y()) + "," + std::to_string(pose.Pos().Z()) + "," +
      std::to_string(pose.Rot().Roll()) + "," + std::to_string(pose.Rot().Pitch()) + "," +
      std::to_string(pose.Rot().Yaw()) + "," + std::to_string(this->update_period) + "\n";
  return pose_msg;
}

void PoseToFilePrivate::CheckUpdatePeriod(const gz::sim::UpdateInfo &_info)
{
  // Get Time step size (Note that duration has to be parsed to a double)
  std::chrono::duration<double> e = _info.dt;
  double step_size = e.count(); 
  
  if (this->update_period < step_size)
  {
    this->update_period = step_size;
    gzerr << "Update period too small. Period set to fit current step size. New period = " << update_period << std::endl;
  }
  else if ( std::fmod(this->update_period,step_size) != 0)
  {
    // Round down to nearest multiple of step size:
    // Round_Down(Number/round_value) * round_value -> where round_value is the number you want to round to -> this case step size
    this->update_period = std::floor(this->update_period/step_size) * step_size;
    gzerr << "Update period not a multiple of step size. Period rounded to nearest step size multiple. New period = " << update_period << std::endl;
  }
}
//////////////////////////////////////////////////

GZ_ADD_PLUGIN(PoseToFile,
              gz::sim::System,
              PoseToFile::ISystemConfigure,
              PoseToFile::ISystemPreUpdate,
              PoseToFile::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(PoseToFile,
                    "gz::sim::systems::PoseToFile")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(PoseToFile,
                    "ignition::gazebo::systems::PoseToFile")
