#ifndef QTICUBSKINGUIPLUGIN_PLUGIN_H
#define QTICUBSKINGUIPLUGIN_PLUGIN_H

#include <QQmlExtensionPlugin>

class QtiDynTreeSoleGuiPluginPlugin : public QQmlExtensionPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.qt-project.Qt.QQmlExtensionInterface")

public:
    void registerTypes(const char *uri);
};

#endif // QTICUBSKINGUIPLUGIN_PLUGIN_H

