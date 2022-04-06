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
// See the License for the specific674b3e6 language governing permissions and
// limitations under the License.

#include "visual_behavior/FollowPerson.h"
#include "visual_behavior/PIDController.h"
#include <string>

namespace visual_behavior
{

FollowPerson::FollowPerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), linear_pid_(0.0, 1.0, 0.0, 0.3), angular_pid_(0.0, 1.0, 0.0, 0.5)
{
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
}

void
FollowPerson::halt()
{
  ROS_INFO("FollowPerson halt");
}

BT::NodeStatus
FollowPerson::tick()
{
  int counter = 0;
  ROS_INFO("FollowPerson tick");

  std::string person_x = getInput<std::string>("person_x").value();
  std::string person_z = getInput<std::string>("person_z").value();

  ROS_INFO("x:%s z:%s", person_x.c_str(), person_z.c_str());
  int x = std::stoi(person_x.c_str());
  double z = std::stod(person_z.c_str());

  geometry_msgs::Twist cmd;
  angular_pid_.set_pid(0.4, 0.05, 0.55);
  linear_pid_.set_pid(0.4, 0.05, 0.55);
  if (x > 320)
  {
    cmd.angular.z = -angular_pid_.get_output(x - 320);
  } else {
    cmd.angular.z = angular_pid_.get_output(320 - x);
  }
  cmd.linear.x = linear_pid_.get_output(z-1);

  ROS_INFO("x: %d = %lf\t z: %lf = %lf", x, cmd.angular.z, z-1, cmd.linear.x);
  pub_vel_.publish(cmd);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::FollowPerson>("FollowPerson");
}
