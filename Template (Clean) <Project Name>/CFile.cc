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

// --- Own includes
#include <chrono>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/header.pb.h>
#include <iostream>
// ---- End

#include <gz/msgs/twist.pb.h>

#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

/////////////////////////////////// NBNBNBNBNBNBNBNBNBNB ////////////////////////////////
/*
Replace all 'HFile_' with the name of the .hh file
Replace all 'CFile_' with the name of the .cc file
*/
#include "HFile_.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::CFile_Private
{

/// \brief Model interface
public:
    Model model{kNullEntity};
  // -------------------------------------- Publisher Node
public:
  transport::Node::Publisher stopPublisher;
};

//////////////////////////////////////////////////
CFile_::CFile_()
    : dataPtr(std::make_unique<CFile_Private>())
{
}

//////////////////////////////////////////////////
void CFile_::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  
}

//////////////////////////////////////////////////
void CFile_::PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("CFile_::PreUpdate");

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
}

//////////////////////////////////////////////////
void CFile_::PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm)
{
  GZ_PROFILE("CFile_::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;


}

//////////////////////////////////////////////////

//////////////////////////////////////////////////

GZ_ADD_PLUGIN(CFile_,
              gz::sim::System,
              CFile_::ISystemConfigure,
              CFile_::ISystemPreUpdate,
              CFile_::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(CFile_,
                    "gz::sim::systems::CFile_")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(CFile_,
                    "ignition::gazebo::systems::CFile_")
