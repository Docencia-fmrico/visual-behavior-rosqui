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

#include <string>

#include "visual_behavior/PercieveBall.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace visual_behavior
{

PercieveBall::PercieveBall(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  nh_()
{
    ROS_INFO("PERCIEVE BALL");
}

void
PercieveBall::halt()
{
  ROS_INFO("PercieveBall halt");
}

BT::NodeStatus
PercieveBall::tick()
{   
    ROS_INFO("PERCIEVE BALL TICK");
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener(buffer_);
    geometry_msgs::TransformStamped bf2ball_msg;
    tf2::Stamped<tf2::Transform> bf2ball;
    std::string error;
    if (buffer_.canTransform("base_footprint", "object/0", ros::Time(0), ros::Duration(0.1), &error))
    {
      bf2ball_msg = buffer_.lookupTransform("base_footprint", "object/0", ros::Time(0));

      tf2::fromMsg(bf2ball_msg, bf2ball);

      double dist = bf2ball.getOrigin().length();
      double angle = atan2(bf2ball.getOrigin().y(), bf2ball.getOrigin().x());

      ROS_INFO("base_footprint -> ball [%lf, %lf] dist = %lf  angle = %lf       %lf ago",
        bf2ball.getOrigin().x(),
        bf2ball.getOrigin().y(),
        dist,
        angle,
        (ros::Time::now() - bf2ball.stamp_).toSec());

        geometry_msgs::Twist vel_msgs;

        if (dist > 2)
          dist = 2;

        vel_msgs.linear.x = dist - 1.0; 
        vel_msgs.angular.z = angle;

        setOutput("ball_x", dist);
        setOutput("ball_z", angle);

        return BT::NodeStatus::SUCCESS;
    }
    else
    {
      ROS_ERROR("%s", error.c_str());
      return BT::NodeStatus::FAILURE;
    }

}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::PercieveBall>("PercieveBall");
}
