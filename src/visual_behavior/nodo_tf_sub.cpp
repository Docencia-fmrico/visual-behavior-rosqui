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

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "geometry_msgs/Twist.h"

#include "geometry_tf/transforms.h"

#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_sub");
  ros::NodeHandle n;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    geometry_msgs::TransformStamped bf2ball_msg;
    tf2::Stamped<tf2::Transform> bf2ball;
    std::string error;

    if (buffer.canTransform("base_footprint", "object/0", ros::Time(0), ros::Duration(0.1), &error))
    {
      bf2ball_msg = buffer.lookupTransform("base_footprint", "object/0", ros::Time(0));

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
        vel_msgs.linear.x = dist - 1.0; 
        vel_msgs.angular.z = angle;

        vel_pub.publish(vel_msgs); 
    }
    else
    {
      ROS_ERROR("%s", error.c_str());
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
