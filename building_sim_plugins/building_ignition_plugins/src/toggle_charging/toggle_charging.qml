/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

ToolBar {
  Layout.minimumWidth: 280
  Layout.minimumHeight: 370

  background: Rectangle {
    color: "transparent"
  }

  ButtonGroup {
    id: group
  }

  GridLayout {
    anchors.fill: parent
    columns: 1
    rows: 3
    columnSpacing: 5
    CheckBox {
      text: qsTr("Allow Charging")
      Layout.columnSpan: 1
      Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
      Layout.leftMargin: 2
      checked: true
      onClicked: {
        toggle_charging.OnEnableCharge(checked)
      }
    }

    CheckBox {
      text: qsTr("Instant Charging")
      Layout.columnSpan: 1
      Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
      Layout.leftMargin: 2
      checked: false
      onClicked: {
        toggle_charging.OnEnableInstantCharge(checked)
      }
    }

    CheckBox {
      text: qsTr("Allow Battery Drain")
      Layout.columnSpan: 1
      Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
      Layout.leftMargin: 2
      checked: true
      onClicked: {
        toggle_charging.OnEnableDrain(checked)
      }
    }

  }
}