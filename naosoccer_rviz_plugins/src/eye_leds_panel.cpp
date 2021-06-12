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

#include "naosoccer_rviz_plugins/eye_leds_panel.hpp"
#include <class_loader/class_loader.hpp>

// One revolution is 360 * 16 increments
// (https://doc.qt.io/qt-5/qpainter.html#drawArc)
#define DEG_TO_QT_ANGLE(deg) (deg * 16)

#define SPAN_ANGLE_PER_LED DEG_TO_QT_ANGLE(45)
#define HALF_SPAN_ANGLE_PER_LED (SPAN_ANGLE_PER_LED / 2.0)

EyeLedsPanel::EyeLedsPanel(QWidget * parent)
: Panel(parent)
{
}

EyeLedsPanel::~EyeLedsPanel() = default;

void EyeLedsPanel::onInitialize()
{
  parentWidget()->setVisible(true);
}

void EyeLedsPanel::paintEvent(QPaintEvent * e)
{
  // We must read in the parameter, to prevent cpplint complaining.
  // https://github.com/ros/roslint/issues/54#issue-178869674
  (void) e;

  QPainter painter(this);

  nao_interfaces::msg::EyeLeds leds;
  drawEyes(painter, leds);
}

void EyeLedsPanel::drawEyes(QPainter & painter, const nao_interfaces::msg::EyeLeds & leds)
{
  QPoint eyesCentre(width() / 2, height() / 2);

  painter.save();
  painter.translate(eyesCentre);

  painter.save();
  float ratio = (float) height() / width();
  if (ratio > 0.5) {
    // Width is limiting factor
    painter.scale(width() / 200.0, width() / 200.0);
  } else {
    // Height is limiting factor
    painter.scale(height() / 100.0, height() / 100.0);
  }

  QPoint reyeCentre(-70, 0);
  QPoint leyeCentre(70, 0);

  QRect eye_rect(QPoint(-30, -30), QPoint(30, 30));

  painter.save();
  painter.translate(reyeCentre);
  // Angles in QT start at 3 o'clock, and goes anti-clockwise.
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(0), leds.leds.at(leds.R2));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(45), leds.leds.at(leds.R1));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(90), leds.leds.at(leds.R0));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(135), leds.leds.at(leds.R7));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(180), leds.leds.at(leds.R6));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(225), leds.leds.at(leds.R5));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(270), leds.leds.at(leds.R4));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(315), leds.leds.at(leds.R3));
  painter.restore();

  painter.save();
  painter.translate(leyeCentre);
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(0), leds.leds.at(leds.L6));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(45), leds.leds.at(leds.L7));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(90), leds.leds.at(leds.L0));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(135), leds.leds.at(leds.L1));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(180), leds.leds.at(leds.L2));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(225), leds.leds.at(leds.L3));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(270), leds.leds.at(leds.L4));
  drawEyeLed(painter, eye_rect, DEG_TO_QT_ANGLE(315), leds.leds.at(leds.L5));
  painter.restore();

  painter.restore();

  painter.restore();
}

void EyeLedsPanel::drawEyeLed(
  QPainter & painter, const QRect & rect, int led_qt_angle,
  const std_msgs::msg::ColorRGBA & color)
{
  int alpha = (color.r + color.g + color.b) * 255 / 3;
  QColor qcolor(color.r * 255, color.g * 255, color.b * 255, alpha);
  QPen pen(QBrush(qcolor), 16, Qt::SolidLine, Qt::FlatCap);
  painter.setPen(pen);
  painter.drawArc(rect, led_qt_angle - HALF_SPAN_ANGLE_PER_LED, SPAN_ANGLE_PER_LED);
}

void EyeLedsPanel::onEnable()
{
  show();
  parentWidget()->show();
}

void EyeLedsPanel::onDisable()
{
  hide();
  parentWidget()->hide();
}

CLASS_LOADER_REGISTER_CLASS(EyeLedsPanel, rviz_common::Panel)
