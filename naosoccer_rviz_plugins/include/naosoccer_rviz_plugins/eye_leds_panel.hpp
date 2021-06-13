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

#ifndef NAOSOCCER_RVIZ_PLUGINS__EYE_LEDS_PANEL_HPP_
#define NAOSOCCER_RVIZ_PLUGINS__EYE_LEDS_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <QPainter>
#include "std_msgs/msg/color_rgba.hpp"
#include "nao_interfaces/msg/eye_leds.hpp"

class EyeLedsPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit EyeLedsPanel(QWidget * parent = nullptr);
  ~EyeLedsPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();

private:
  void paintEvent(QPaintEvent * e) override;

  void drawEyes(QPainter & painter, const nao_interfaces::msg::EyeLeds & leds);
  void drawEyeLed(
    QPainter & painter, const QRect & rect, int led_qt_angle,
    const std_msgs::msg::ColorRGBA & color);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nao_interfaces::msg::EyeLeds>::SharedPtr sub_;
  
  nao_interfaces::msg::EyeLeds::SharedPtr leds;
};


#endif  // NAOSOCCER_RVIZ_PLUGINS__EYE_LEDS_PANEL_HPP_
