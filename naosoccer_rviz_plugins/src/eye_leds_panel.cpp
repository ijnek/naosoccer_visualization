#include "naosoccer_rviz_plugins/eye_leds_panel.hpp"
#include <class_loader/class_loader.hpp>

EyeLEDsPanel::EyeLEDsPanel(QWidget* parent) : Panel(parent)
{

}

EyeLEDsPanel::~EyeLEDsPanel() = default;

void EyeLEDsPanel::onInitialize()
{
  parentWidget()->setVisible(true);
}

void EyeLEDsPanel::onEnable()
{
  show();
  parentWidget()->show();
}

void EyeLEDsPanel::onDisable()
{
  hide();
  parentWidget()->hide();
}

CLASS_LOADER_REGISTER_CLASS(EyeLEDsPanel, rviz_common::Panel)