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

#ifndef VISUAL_BEHAVIOR_TURN_H
#define VISUAL_BEHAVIOR_TURN_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <string>
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace visual_behavior
{

class Turn : public BT::ActionNodeBase
{
  public:
    explicit Turn(const std::string& name);

    void halt();

    BT::NodeStatus tick();
  protected:
    ros::NodeHandle nh_;
    ros::Publisher pub_vel_;
    static constexpr double TURN_VEL = 0.35;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_TURN_H
