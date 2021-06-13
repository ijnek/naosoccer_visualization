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
#include <rviz_common/display_context.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

// One revolution is 360 * 16 increments
// (https://doc.qt.io/qt-5/qpainter.html#drawArc)
#define DEG_TO_QT_ANGLE(deg) (deg * 16)

#define SPAN_ANGLE_PER_LED DEG_TO_QT_ANGLE(45)
#define HALF_SPAN_ANGLE_PER_LED (SPAN_ANGLE_PER_LED / 2.0)

EyeLedsPanel::EyeLedsPanel(QWidget * parent)
: Panel(parent)
{
  loadFaceImage();
}

EyeLedsPanel::~EyeLedsPanel() = default;

void EyeLedsPanel::loadFaceImage()
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
    "naosoccer_rviz_plugins");
  image = QImage(QString::fromStdString(package_share_directory + "/images/face.png"));
  imageW = image.width();
  imageH = image.height();
}

QSize EyeLedsPanel::sizeHint() const
{
  return QSize(imageW, imageH);
}

void EyeLedsPanel::onInitialize()
{
  parentWidget()->setVisible(true);

  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  sub_ = node_->create_subscription<nao_interfaces::msg::EyeLeds>(
    "effectors/eye_leds", 1,
    [this](nao_interfaces::msg::EyeLeds::SharedPtr eye_leds) {
      this->leds = eye_leds;
      this->update();  // QWidget method which redraws widget
    });
}

void EyeLedsPanel::paintEvent(QPaintEvent * e)
{
  // We must read in the parameter, to prevent cpplint complaining.
  // https://github.com/ros/roslint/issues/54#issue-178869674
  (void) e;

  if (!leds) {  // If we haven't received led information yet
    return;
  }

  QPainter painter(this);

  painter.save();
  float widgH = static_cast<float>(height());
  float widgW = static_cast<float>(width());
  painter.translate(widgW / 2, widgH / 2);
  float ratio = widgH / widgW;
  float scale = 1.0;
  if (ratio > imageH / imageW) {
    // Width is limiting factor
    scale = widgW / imageW;
  } else {
    // Height is limiting factor
    scale = widgH / imageH;
  }
  painter.scale(scale, scale);

  painter.drawImage(QRectF(-imageW / 2, -imageH / 2, imageW, imageH), image);
  drawEyes(painter, *leds);

  painter.restore();
}

void EyeLedsPanel::drawEyes(QPainter & painter, const nao_interfaces::msg::EyeLeds & leds)
{
  QPoint reyeCentre(-33, 5);
  QPoint leyeCentre(33, 5);

  QRect eye_rect(QPoint(-9, -9), QPoint(9, 9));

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
}

void EyeLedsPanel::drawEyeLed(
  QPainter & painter, const QRect & rect, int led_qt_angle,
  const std_msgs::msg::ColorRGBA & color)
{
  int alpha = (color.r + color.g + color.b) * 255 / 3;
  QColor qcolor(color.r * 255, color.g * 255, color.b * 255, alpha);
  QPen pen(QBrush(qcolor), 8, Qt::SolidLine, Qt::FlatCap);
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
