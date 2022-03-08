// Copyright 2022 Intelligent Robotics Lab
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

#include "visual_behavior/follow_ball.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace visual_behavior
{

// Constructor
Follow_Ball::Follow_Ball()
{
  /* sub_ = n_.subscribe("/darknet_ros/bouding_boxes", 1, &Follow_Ball::Callback, this);
  pub_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1); */
}

void
Follow_Ball::Callback()
{
   ROS_INFO("CALLBACK\n");
}

void
Follow_Ball::step()
{
    ROS_INFO("STEP\n");
}

}  // namespace visual_behavior
