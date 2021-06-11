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
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define VISUALISATION_BALL_DIAMETER 0.1  // m

class BallToMarker : public rclcpp::Node
{
public:
  BallToMarker()
  : Node("BallToMarker")
  {
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "visualization/ball", 10);
    subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "vision/ball", 1,
      [this](geometry_msgs::msg::PointStamped::SharedPtr point) {
        publisher_->publish(convert(*point));
      });
  }

private:
  visualization_msgs::msg::Marker convert(geometry_msgs::msg::PointStamped & point)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = point.header.frame_id;
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.point.x;
    marker.pose.position.y = point.point.y;
    marker.pose.position.z = point.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = VISUALISATION_BALL_DIAMETER;
    marker.scale.y = VISUALISATION_BALL_DIAMETER;
    marker.scale.z = VISUALISATION_BALL_DIAMETER;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    return marker;
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;
};

int
main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallToMarker>());
  rclcpp::shutdown();
  return 0;
}
