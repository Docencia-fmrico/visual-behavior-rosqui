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

#ifndef FOLLOW_PERSON_H
#define FOLLOW_PERSON_H

//Clase padre

#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

namespace visual_behavior
{

class Follow_Person
{
public:
  Follow_Person();

  void Callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
  void step();

protected:
  ros::NodeHandle n_;

  ros::Subscriber sub_;
  ros::Publisher pub_;
};

}  // namespace visual_behavior

#endif  // FOLLOW_PERSON_H
