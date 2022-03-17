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

#ifndef VISUAL_BEHAVIOR_PERCIEVEBALL_H
#define VISUAL_BEHAVIOR_PERCIEVEBALL_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "geometry_msgs/Twist.h"

#include "visual_behavior/transforms.h"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>

#include <string>

namespace visual_behavior
{

class PercieveBall : public BT::ActionNodeBase
{
  public:
    explicit PercieveBall(const std::string& name, const BT::NodeConfiguration & config);

    void halt();

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("ball_z"), BT::OutputPort<std::string>("ball_x")};
    }

    BT::NodeStatus tick();

  private:
    ros::NodeHandle nh_;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_PERCIEVEBALL_H
