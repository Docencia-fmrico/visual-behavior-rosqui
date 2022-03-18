
// Copyright 2022 ROSqui
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "visual_behavior/Turn.h"

namespace visual_behavior
{

Turn::Turn(const std::string& name)
: BT::ActionNodeBase(name, {})
{
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",100);
}

void
Turn::halt()
{
  ROS_INFO("Turn halt");
}

BT::NodeStatus
Turn::tick()
{
  //ROS_INFO("Turn tick");
  geometry_msgs::Twist cmd;

  cmd.linear.x = 0;
  cmd.angular.z = TURN_VEL;

  pub_vel_.publish(cmd);
  
  return BT::NodeStatus::FAILURE;
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::Turn>("Turn");
}
