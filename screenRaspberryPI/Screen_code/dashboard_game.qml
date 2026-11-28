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

        // --- LEFT ARM ---
        ColumnLayout {
            Layout.preferredWidth: parent.width * 0.3
            Layout.fillHeight: true
            spacing: 0
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#ee22cc"
                border.color: "#2c3e50"
                Image{
                    source : {
                        if (backend.left_arm === 1) {
                            //arm by Jacqueline Fernandes from Noun Project (CC BY 3.0)
                            return "pics/arm_raised_left.png" ;
                        } else {
                            // arm by Harianto from Noun Project (CC BY 3.0)
                            return "pics/arm_low_left.png" ;
                        }
                    }
                    width: parent.width
                    height: parent.height
                    fillMode: Image.Stretch
                }
            }
        }

        // --- INSTRUCTIONS ---
        ColumnLayout {
            Layout.preferredWidth: parent.width * 0.4
            Layout.fillHeight: true
            spacing: 0

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: {
                    if (backend.gameState === "Deactivated") {
                        return "#34495e";
                    } else if (backend.gameState === "Oops, it looks like you didn't do the right moves... ") {
                        return '#c02121';
                    } else if (backend.gameState === "Follow the car's movement's !") {
                        return '#d66e28';
                    } else if (backend.gameState === "Now do the same moves as the car") {
                        return '#5354c5';
                    } else if (backend.gameState === "Good job ! Can you do it again with one more move ?") {
                        return '#15bd31';
                    } else {
                        return '#9d2bc0'; // Error
                    }
                }
                border.color: "#2c3e50"

                ColumnLayout {
                    anchors.centerIn: parent

                    Text {
                        text: backend.gameState
                        color: "white"
                        font.pixelSize: 60
                        Layout.alignment: Qt.AlignHCenter
                        // Layout.topMargin: 60
                    }
                }
            }
        }

        // --- RIGHT ARM ---
        ColumnLayout {
            Layout.preferredWidth: parent.width * 0.3
            Layout.fillHeight: true
            spacing: 0
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#ee22cc"
                border.color: "#2c3e50"
                Image{
                    source : {
                        if (backend.right_arm === 1) {
                            //arm by Jacqueline Fernandes from Noun Project (CC BY 3.0)
                            return "pics/arm_raised_right.png" ;
                        } else {
                            // arm by Harianto from Noun Project (CC BY 3.0)
                            return "pics/arm_low_right.png" ;
                        }
                    }
                    width: parent.width
                    height: parent.height
                    fillMode: Image.Stretch
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
