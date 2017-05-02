#ifndef QTICUBSKINGUIPLUGIN_H
#define QTICUBSKINGUIPLUGIN_H

#include <QtQuick>
#include <QPainter>
#include <QTimer>

#include <yarp/os/all.h>

#include "include/ObserverThread.h"


class QtiDynTreeSoleGuiPlugin : public QQuickPaintedItem
{
    Q_OBJECT
    Q_DISABLE_COPY(QtiDynTreeSoleGuiPlugin)
    Q_PROPERTY(QString windowTitle READ windowTitle NOTIFY windowTitleChanged)
    Q_PROPERTY(int posX READ posX NOTIFY posXChanged)
    Q_PROPERTY(int posY READ posY NOTIFY posYChanged)
    Q_PROPERTY(int windowWidth READ windowWidth NOTIFY widthChanged)
    Q_PROPERTY(int windowHeight READ windowHeight NOTIFY heightChanged)

public:
    QtiDynTreeSoleGuiPlugin(QQuickItem *parent = 0);
    ~QtiDynTreeSoleGuiPlugin();
    Q_INVOKABLE bool parseParameters(QStringList params);

    void paint(QPainter *painter);

    QString windowTitle();
    int posX();
    int posY();
    int windowWidth();
    int windowHeight();

private:
    yarp::os::ResourceFinder rf;
    QMutex mutex;
    int gWidth;
    int gHeight;
    int gRowStride;
    int gImageSize;
    int gMapSize;
    int gImageArea;
    int gXpos;
    int gYpos;
    bool bDrawing;
    enum   TheadTypeEnum {TYPE_CAN, TYPE_PORT};
    double *gpActivationMap;
    ObserverThread *observerThread;

    QTimer timer;
    QString window_title;

signals:
    void init();
    void sendWindowTitle(QString title);
    void sendWindowSize(int width, int height);
    void sendWindowPos(int xPos, int yPos);
    void done();


    void posXChanged();
    void posYChanged();
    void widthChanged();
    void heightChanged();
    void windowTitleChanged();

private slots:
    void onTimeout();

    void onInit();
};

#endif // QTICUBSKINGUIPLUGIN_H

