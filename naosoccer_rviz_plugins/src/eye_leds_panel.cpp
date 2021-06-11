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

EyeLEDsPanel::EyeLEDsPanel(QWidget * parent)
: Panel(parent)
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
