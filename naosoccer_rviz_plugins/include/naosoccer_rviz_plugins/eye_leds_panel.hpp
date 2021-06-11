#ifndef NAOSOCCER_RVIZ_PLUGINS__EYE_LEDS_PANEL_HPP_
#define NAOSOCCER_RVIZ_PLUGINS__EYE_LEDS_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

class EyeLEDsPanel: public rviz_common::Panel
{
  Q_OBJECT
public:
  EyeLEDsPanel(QWidget* parent = nullptr);
  ~EyeLEDsPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();
};


#endif  // NAOSOCCER_RVIZ_PLUGINS__EYE_LEDS_PANEL_HPP_