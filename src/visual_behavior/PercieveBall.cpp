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
  nh_(),
  image_depth_sub(nh_, "/camera/depth/image_raw", 1),
  bbx_sub(nh_, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub)
{
  detected = false;
  sync_bbx.registerCallback(boost::bind(&PercieveBall::callback_bbx, this, _1, _2));
}

void PercieveBall::callback_bbx(const sensor_msgs::ImageConstPtr& image,
const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{

  cv_bridge::CvImagePtr img_ptr_depth;
  
  try{
      img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
  }

  int px = 0;
  int py = 0;
  float dist = 0;
  // Darknet only detects person
  for (const auto & box : boxes->bounding_boxes) {
    ROS_INFO("PROB: %f", box.probability);
    if ((box.probability > 0.5))
    {
      ROS_INFO("DETECTED TRUE");
      detected = true;
      px = (box.xmax + box.xmin) / 2;
      py = (box.ymax + box.ymin) / 2;

      dist = img_ptr_depth->image.at<float>(cv::Point(px, py))*0.001f;

      ROS_INFO("person_x: %d \t person_z: %f\n", px, dist);
    }
  }
  setOutput("person_x", px);
  setOutput("person_z", dist);
}

void
PercieveBall::halt()
{
  ROS_INFO("PercievePerson halt");
}

BT::NodeStatus
PercieveBall::tick()
{
  ros::spinOnce();
  if ( detected )
  {
    detected = false;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    //ROS_INFO("Detected: FALSE");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::PercieveBall>("PercieveBall");
}