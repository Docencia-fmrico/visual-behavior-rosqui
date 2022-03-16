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

namespace visual_behavior
{

FollowBall::FollowBall(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",100);
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

    std::string person_x = getInput<std::string>("person_x").value();
    std::string person_z = getInput<std::string>("person_z").value();

    ROS_INFO("X:%d Z:%d",person_x, person_z);
    
    /*geometry_msgs::Twist cmd;
    if (person_x < 235)
      cmd.angular.z = -0.4;
    else if (person_x > 305)
      cmd.angular.z = 0.4;
    else 
      cmd.angular.z = 0;
    
    /*if (std::stod(person_z) > 2)
      cmd.linear.x = 0.4;
    else
      cmd.linear.x = 0;
   
    pub_vel_.publish(cmd);*/


    return BT::NodeStatus::SUCCESS;
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::FollowBall>("FollowBall");
}
