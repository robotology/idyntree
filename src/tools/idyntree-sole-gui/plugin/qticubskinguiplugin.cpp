/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
*
**/

#include "qticubskinguiplugin.h"
#include <QDebug>

using namespace yarp::os;


static yarp::os::Semaphore gMutex(1);

QtiDynTreeSoleGuiPlugin::QtiDynTreeSoleGuiPlugin(QQuickItem *parent):
    QQuickPaintedItem(parent)
{
    // By default, QQuickItem does not draw anything. If you subclass
    // QQuickItem to create a visual item, you will need to uncomment the
    // following line and re-implement updatePaintNode()

     setFlag(ItemHasContents, true);

     gXpos = 32;
     gYpos = 32;

     gWidth = 300;
     gHeight = 300;
     gRowStride = 0;
     gImageArea = 0;
     gImageSize = 0;
     gMapSize = 0;

     gpActivationMap = NULL;
     observerThread = NULL;

     window_title = "SoleGui";

     timer.setInterval(50);
     timer.setSingleShot(false);
     connect(&timer, SIGNAL(timeout()), this, SLOT(onTimeout()));

}

QtiDynTreeSoleGuiPlugin::~QtiDynTreeSoleGuiPlugin()
{
    timer.stop();
    mutex.lock();

    if (observerThread){
        observerThread->stop();
        delete observerThread;
    }

    mutex.unlock();
}


void QtiDynTreeSoleGuiPlugin::paint(QPainter *painter)
{

    if (!timer.isActive()){
        return;
    }

    int width=painter->device()->width();
    int height=painter->device()->height();


   bDrawing=false;

   if (bDrawing){
       return;
   }

   bDrawing=true;

   mutex.lock();
    if (width!=gWidth || height!=gHeight){
        gWidth=width;
        gHeight=height;

        gRowStride=3*gWidth;
        gImageSize=gRowStride*gHeight;
        gImageArea=gWidth*gHeight;
        gMapSize=gWidth*gHeight*sizeof(double);

        if (gpActivationMap){
            delete [] gpActivationMap;
        }

        gpActivationMap = new double[gImageArea];

        if (observerThread && gWidth>=180 && gHeight>=180){
            //gpSkinMeshThreadPort->resize(gWidth,gHeight);
        }
    }

    if (observerThread) {

        painter->beginNativePainting();
        observerThread->draw(painter, gWidth, gHeight);
        painter->endNativePainting();
    }

    mutex.unlock();
    bDrawing=false;

}


/*! \brief parse the parameters received from the main container in QstringList form
    \param params the parameter list
*/
bool QtiDynTreeSoleGuiPlugin::parseParameters(QStringList params)
{
    Network yarp;

    rf.setVerbose();
    rf.setDefaultContext("soleGui/soleGui");
    rf.setDefaultConfigFile("soleGui.ini");

    // Transform Qt Params array in standard argc & argv
    int c = params.count();
    char **v;
    v = (char**)malloc(sizeof(char*) * c);
    for (int i=0; i<params.count(); i++){
        v[i] = (char*)malloc(sizeof(char) * params.at(i).length()+1);
        strcpy(v[i], params.at(i).toLatin1().data());
    }

    if (!rf.configure(c, v)){
        free(v);
        return false;
    }

    gWidth =rf.find("width" ).asInt();
    gHeight=rf.find("height").asInt();
    if (rf.check("xpos")){
        gXpos=rf.find("xpos").asInt();
    }
    if (rf.check("ypos")){
        gYpos=rf.find("ypos").asInt();
    }

    window_title=QString("SoleGui");
    windowTitleChanged();
    posXChanged();
    posYChanged();
    widthChanged();
    heightChanged();

    yDebug("RF: %s", rf.toString().data());

    gRowStride=3*gWidth;
    gImageSize=gRowStride*gHeight;
    gMapSize=gWidth*gHeight*sizeof(double);
    gImageArea=gWidth*gHeight;

    gpActivationMap=new double[gImageArea];

    /***********/
    // TODO
    /***********/

    timer.start();

    connect(this, SIGNAL(init()), this, SLOT(onInit()), Qt::QueuedConnection);
    init();


    return true;
}


void QtiDynTreeSoleGuiPlugin::onTimeout()
{
    update();
}

void QtiDynTreeSoleGuiPlugin::onInit()
{
    int period = rf.check("period")?rf.find("period").asInt():50;

    observerThread=new ObserverThread(rf, period);
    observerThread->start();

    done();
}

/*! \brief Returns the title.*/
QString QtiDynTreeSoleGuiPlugin::windowTitle()
{
    return window_title;
}

/*! \brief Returns the x position.*/
int QtiDynTreeSoleGuiPlugin::posX()
{
    return gXpos;
}

/*! \brief Returns the y position.*/
int QtiDynTreeSoleGuiPlugin::posY()
{
    return gYpos;
}

/*! \brief Returns the width.*/
int QtiDynTreeSoleGuiPlugin::windowWidth()
{
    return gWidth;
}

/*! \brief Returns the height.*/
int QtiDynTreeSoleGuiPlugin::windowHeight()
{
    return gHeight;
}
