// Copyright 2021 Kenji Brameld
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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "soccer_vision_msgs/msg/goalpost_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class GoalpostArrayToMarkerArray : public rclcpp::Node
{
public:
  GoalpostArrayToMarkerArray()
  : Node("GoalpostArrayToMarkerArray")
  {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "visualization/goalposts", 10);
    subscriber_ = this->create_subscription<soccer_vision_msgs::msg::GoalpostArray>(
      "vision/goalposts", 1,
      [this](soccer_vision_msgs::msg::GoalpostArray::SharedPtr goalpostArray) {
        visualization_msgs::msg::MarkerArray markerArray;
        markerArray.markers.push_back(createDeleteAllActionMarker());
        for (unsigned i = 0; i < goalpostArray->posts.size(); ++i) {
          markerArray.markers.push_back(convert(i, goalpostArray->posts.at(i)));
        }
        publisher_->publish(markerArray);
      });
  }

private:
  visualization_msgs::msg::Marker convert(int markerId, soccer_vision_msgs::msg::Goalpost & post)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = post.header;
    marker.ns = "";
    marker.id = markerId;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = post.point.x;
    marker.pose.position.y = post.point.y;
    marker.pose.position.z = post.point.z;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    if (post.observed_top) {
      marker.mesh_resource = "package://naosoccer_visualization/meshes/goalpost_observed_top.dae";
    } else {
      marker.mesh_resource = "package://naosoccer_visualization/meshes/goalpost_observed_bot.dae";
    }

    return marker;
  }

  visualization_msgs::msg::Marker createDeleteAllActionMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<soccer_vision_msgs::msg::GoalpostArray>::SharedPtr subscriber_;
};

int
main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalpostArrayToMarkerArray>());
  rclcpp::shutdown();
  return 0;
}
