
// Copyright 2019 Intelligent Robotics Lab
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

#include <string>

#include "visual_behavior/FollowPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace visual_behavior
{

FollowPerson::FollowPerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0)
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void
FollowPerson::halt()
{
  ROS_INFO("FollowPerson halt");
}

BT::NodeStatus
FollowPerson::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("First time in FollowPerson");
  }

  std::string object = getInput<std::string>("object").value();
  ROS_INFO("FollowPerson [%s] tick %d", object.c_str(), counter_);

  if (counter_++ < 50)
  {
    geometry_msgs::Twist msg;
    msg.linear.x = 0.4;

    vel_pub_.publish(msg);
    return BT::NodeStatus::RUNNING;
  }
  else
  {
    geometry_msgs::Twist msg;
    vel_pub_.publish(msg);

    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::FollowPerson>("FollowPerson");
}
