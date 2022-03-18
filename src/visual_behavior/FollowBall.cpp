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

#include "visual_behavior/FollowBall.h"
#include <string>

namespace visual_behavior
{

FollowBall::FollowBall(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
}

void
FollowBall::halt()
{
  ROS_INFO("FollowBall halt");
}

BT::NodeStatus
FollowBall::tick()
{
    ROS_INFO("FollowBall tick");

    std::string ball_x = getInput<std::string>("ball_x").value();
    std::string ball_z = getInput<std::string>("ball_z").value();
    int X = std::stoi(ball_x.c_str());
    double Z = std::stod(ball_z.c_str());

    geometry_msgs::Twist vel_msgs_;

    vel_msgs_.angular.z = X;

    vel_msgs_.linear.x = Z;

    vel_pub_.publish(vel_msgs_);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::FollowBall>("FollowBall");
}
