import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

ApplicationWindow {
    visible: true
    visibility: Window.Maximized
    //width: 800
    //height: 480
    title: backend.motd // "Car Dashboard"
    color: "#2c3e50"

    RowLayout {
        anchors.fill: parent
        spacing: 0

        // --- LEFT SIDE: STATE ---
        ColumnLayout {
            Layout.preferredWidth: parent.width * 0.65
            Layout.fillHeight: true
            spacing: 0

            // Mode
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: backend.state === 1 ? "#27ae60" : "#c0392b"
                border.color: "#2c3e50"

                ColumnLayout {
                    anchors.centerIn: parent

                    Text {
                        text: backend.state === 1 ? "RUNNING" : "STOPPED"
                        color: "white"
                        font.pixelSize: 80
                        font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }

                    Text {
                        text: backend.state === 1 ? "System Active" : "Waiting..."
                        color: "#ecf0f1"
                        font.pixelSize: 30
                        Layout.alignment: Qt.AlignHCenter
                    }
                }
            }

            // Gesture
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                // color: backend.state === 1 ? "#27ae60" : "#c0392b"
                color: {
                    if (backend.gesture === "none") {
                        return "#34495e";
                    } else if (backend.gesture === "Victory") {
                        return "#f39c12";
                    } else if (backend.gesture === "Pointing Up") {
                        return '#dde00f';
                    } else if (backend.gesture === "Thumb Up") {
                        return "#27ae60";
                    } else if (backend.gesture === "Rock n Roll") {
                        return "#c0392b";
                    } else {
                        return '#9d2bc0';
                    }
                }
                border.color: "#2c3e50"

                ColumnLayout {
                    anchors.centerIn: parent

                    Text {
                        text: "Detected gesture"
                        color: "white"
                        font.pixelSize: 30
                        Layout.alignment: Qt.AlignHCenter
                        // Layout.topMargin: 60
                    }

                    Text {
                        text: backend.gesture
                        color: "white"
                        font.pixelSize: 60
                        font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }
                }
            }

            // Follow State
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                // color: backend.state === 1 ? "#27ae60" : "#c0392b"
                color: {
                    if (backend.followingState === "Deactivated") {
                        return "#34495e";
                    } else if (backend.followingState === "Master lost") {
                        return '#c02121';
                    } else if (backend.followingState === "Looking for master") {
                        return '#d66e28';
                    } else if (backend.followingState === "Waiting gesture") {
                        return "#003049";
                    } else if (backend.followingState === "Change master") {
                        return '#5354c5';
                    } else if (backend.followingState === "Stop following") {
                        return '#1bdaa0';
                    } else if (backend.followingState === "Following master") {
                        return '#15bd31';
                    } else {
                        return '#9d2bc0'; // Error
                    }
                }
                border.color: "#2c3e50"

                ColumnLayout {
                    anchors.centerIn: parent

                    Text {
                        text: "State"
                        color: "white"
                        font.pixelSize: 30
                        Layout.alignment: Qt.AlignHCenter
                        // Layout.topMargin: 60
                    }

                    Text {
                        text: backend.followingState
                        color: "white"
                        font.pixelSize: 60
                        font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }
                }
            }
        }

        // --- RIGHT SIDE: SENSORS ---
        ColumnLayout {
            Layout.preferredWidth: parent.width * 0.35
            Layout.fillHeight: true
            spacing: 0

            // Temperature
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#34495e"
                border.color: "#2c3e50"

                ColumnLayout {
                    anchors.centerIn: parent

                    Text {
                        text: "TEMPERATURE"
                        color: "#95a5a6"
                        font.pixelSize: 50
                        Layout.alignment: Qt.AlignHCenter
                    }
                    Text {
                        text: backend.temperature + " °C"
                        color: "white"
                        font.pixelSize: 70
                        font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }
                }
            }

            // Humidity
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#34495e"
                border.color: "#2c3e50"

                ColumnLayout {
                    anchors.centerIn: parent

                    Text {
                        text: "HUMIDITY"
                        color: "#95a5a6"
                        font.pixelSize: 50
                        Layout.alignment: Qt.AlignHCenter
                    }
                    Text {
                        text: backend.humidity + " %"
                        color: "#3498db"
                        font.pixelSize: 70
                        font.bold: true
                        Layout.alignment: Qt.AlignHCenter
                    }
                }
            }
        }
    }

    Text {
        text: backend.motd
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 8
        color: "#7f8c8d"
        font.pixelSize: 14
        opacity: 0.6
    }
}
