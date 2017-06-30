
#include "ChartsManager.h"

#include <QtWidgets/QApplication>
#include <QWindow>
#include <yarp/os/ResourceFinder.h>

int main(int argc, char** argv) {

    QApplication app(argc, argv);
    //Avoid closing app when last window closes
    app.setQuitOnLastWindowClosed(false);

    yarp::os::ResourceFinder& finder = yarp::os::ResourceFinder::getResourceFinderSingleton();
    finder.configure(argc, argv);

    ChartsManager manager;
    if (!manager.initialize()) return -1;

    return app.exec();
}
