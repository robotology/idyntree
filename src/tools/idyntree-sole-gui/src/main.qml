import QtQuick 2.0
import QtQuick.Controls 1.1
import QtQuick.Window 2.1

import robotology.idyntree.solegui 1.0


ApplicationWindow {
    id: window
    visible: false
    width: skinGui.windowWidth
    height: skinGui.windowHeight
    title: skinGui.windowTitle
    x: skinGui.posX
    y: skinGui.posY


    function parseParameters(params){
        var ret = skinGui.parseParameters(params)
        return ret
    }

//    Connections{
//        target: skinGui

//        onSendWindowTitle:{
//            window.title = title
//        }

//        onSendWindowSize:{
//            window.width = width
//            window.height = height
//        }

//        onSendWindowPos:{
//            window.x = xPos
//            window.y = yPos
//        }
//        onDone:{
//            window.visible = true
//        }
//    }


    QtiDynTreeSoleGuiPlugin{
        anchors.fill: parent
        id: skinGui
        objectName: "iDynTreeSoleGui"
    }

}
