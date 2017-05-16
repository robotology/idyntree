#include <QApplication>
#include <QQmlApplicationEngine>
#include <QVariant>
#include <QQuickWindow>
#include <QDir>
#include "config.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QVariant retVal;

    // De-comment this to trace all imports
    QByteArray data = "1";
    qputenv("QML_IMPORT_TRACE", data);

    QQmlApplicationEngine engine;
    engine.addImportPath(QDir::cleanPath(QCoreApplication::applicationDirPath() + QDir::separator() +
                                         PLUGINS_RELATIVE_PATH));
#ifdef CMAKE_INTDIR
    engine.addImportPath(QDir::cleanPath(QCoreApplication::applicationDirPath() + QDir::separator() +
                                         ".." + QDir::separator() +
                                         PLUGINS_RELATIVE_PATH + QDir::separator() +
                                         CMAKE_INTDIR));
#endif

    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    QObject *topLevel = engine.rootObjects().value(0);
    QQuickWindow *window = qobject_cast<QQuickWindow *>(topLevel);

    // Pack the argc and argv to a QStrinList so we can pass them easily to the plugin
    QStringList params;
    for(int i=0;i<argc;i++){
        params.append(argv[i]);
    }
    // Call the parseParameters of the toplevel object
    QMetaObject::invokeMethod(topLevel,"parseParameters",
                              Qt::DirectConnection,
                              Q_RETURN_ARG(QVariant, retVal),
                              Q_ARG(QVariant,params));
    if(!retVal.toBool()){
        return 1;
    }

    if(window){
        window->show();
    }

    return (app.exec()!=0?1:0);
}
