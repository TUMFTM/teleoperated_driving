import QtQuick 2.1

Rectangle {
    id: root
    width: joystick.width
    height: joystick.height
    color: "transparent"
    signal userInput(double x, double y);
    


    Image {
        id: joystick
        property real angle : 0
        property real distance : 0
        source: "images/background.png"
        width: parent.width
        height: parent.height
        anchors.centerIn: parent



        ParallelAnimation {
            id: returnAnimation
            NumberAnimation { target: stick.anchors; property: "horizontalCenterOffset";
                to: stick.anchors.horizontalCenterOffset; duration: 200; easing.type: Easing.OutSine }
            NumberAnimation { target: stick.anchors; property: "verticalCenterOffset";
                to: 0; duration: 200; easing.type: Easing.OutSine }
        }

        MouseArea {
            id: mouse
            focus: true
            property int mcx : mouseX - distanceBoundX
            property int mcy : mouseY - distanceBoundY
            property int keyx : 0
            property int keyy : 0
            property int accel : 5
            property bool inputInBoundsX : Math.abs(mcx) < Math.abs(width*0.5)
            property bool inputInBoundsY : Math.abs(mcy) < Math.abs(height*0.5)
            property bool keyInputInBoundsX : Math.abs(keyx) < Math.abs(width*0.5)
            property bool keyInputInBoundsY : Math.abs(keyy) < Math.abs(height*0.5)
            property bool steerOnlyMode : false
            property bool mousePressed : false
            property real distanceBoundX : width * 0.5
            property real distanceBoundY : height * 0.5
            anchors.fill: parent

            function positionChangedKeyboard() {

                if (keyInputInBoundsX) {
                    stick.anchors.horizontalCenterOffset = keyx
                }
                else {
                    stick.anchors.horizontalCenterOffset = Math.sign(keyx) * distanceBoundX
                }
                if (keyInputInBoundsY) {
                    stick.anchors.verticalCenterOffset = keyy 
                } else {
                    stick.anchors.verticalCenterOffset = Math.sign(keyy) * distanceBoundY
                }
                userInput(
                    stick.anchors.horizontalCenterOffset / distanceBoundX,
                    -stick.anchors.verticalCenterOffset / distanceBoundY
                );

            }


            Keys.onPressed: {
                if (event.key == Qt.Key_Control && event.isAutoRepeat == false && mousePressed == true) {
                    steerOnlyMode = true 
                    returnAnimation.restart()
                    userInput(stick.anchors.horizontalCenterOffset / distanceBoundX, 0);
                    event.accepted = true;
                }

                /*if (event.key == Qt.Key_Up || event.key == Qt.Key_W) {
                    if (!mousePressed && !steerOnlyMode)  {
                        if  (event.isAutoRepeat)
                        returnAnimation.stop()
                        keyy += -accel
                        positionChangedKeyboard()
                        //userInput(1,0)
                        event.accepted = true; 

                    }
                }
                if (event.key == Qt.Key_Down || event.key == Qt.Key_S) {
                    if (!mousePressed && !steerOnlyMode) {
                        returnAnimation.stop()
                        keyy += accel 
                        positionChangedKeyboard()
                        //userInput(-1,0)
                        event.accepted = true; 

                    }
                }*/
                if (event.key == Qt.Key_Left || event.key == Qt.Key_A) {
                    if (!mousePressed) {
                        returnAnimation.stop()
                        keyx += -accel 
                        positionChangedKeyboard()
                        //userInput(0,-1)
                        event.accepted = true; 

                    }
                }
                if (event.key == Qt.Key_Right || event.key == Qt.Key_D) {
                    if (!mousePressed) {
                        returnAnimation.stop()
                        keyx += accel
                        positionChangedKeyboard()
                        //userInput(0,1)
                        event.accepted = true; 

                    }
                }

            }
            Keys.onReleased: {
                if (event.key == Qt.Key_Control && event.isAutoRepeat == false) {
                    console.log("On Release Mouse")
                    if (mousePressed == false) {
                        stick.anchors.verticalCenterOffset = 0;
                    }
                    userInput(
                        stick.anchors.horizontalCenterOffset / distanceBoundX,
                        -stick.anchors.verticalCenterOffset / distanceBoundY
                    );
                    event.accepted = true;
                }
                if (//event.key == Qt.Key_Up || event.key == Qt.Key_Down || event.key == Qt.Key_W || event.key == Qt.Key_S 
                 event.key == Qt.Key_Right || event.key == Qt.Key_Left || event.key == Qt.Key_D || event.key == Qt.Key_A 
                ) {
                    if (mousePressed == false) {
                    returnAnimation.restart()
                    event.accepted = true;
                    }
                }
            }
                
                
            onPressed: {
                mousePressed = true
                returnAnimation.stop();
            }

            onReleased: {
                mousePressed = false
                returnAnimation.restart()
                
                userInput(stick.anchors.horizontalCenterOffset / distanceBoundX, 0);
            }

            onPositionChanged: {
                if (inputInBoundsX) {
                    stick.anchors.horizontalCenterOffset = mcx
                } else {
                    stick.anchors.horizontalCenterOffset = Math.sign(mcx)*distanceBoundX
                }
                if (steerOnlyMode){
                    stick.anchors.verticalCenterOffset = 0
                }else{
                    if (inputInBoundsY) {
                        stick.anchors.verticalCenterOffset = mcy
                    } else {
                        stick.anchors.verticalCenterOffset = Math.sign(mcy)*distanceBoundY
                    }
                }
                userInput(
                    stick.anchors.horizontalCenterOffset / distanceBoundX,
                    -stick.anchors.verticalCenterOffset / distanceBoundY
                );
            }
        }



        Image {
            id: stick
            source: "images/stick.png"
            anchors.centerIn: parent
        }
    }

}
