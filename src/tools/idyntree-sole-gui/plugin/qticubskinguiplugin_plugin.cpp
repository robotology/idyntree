#include "qticubskinguiplugin_plugin.h"
#include "qticubskinguiplugin.h"

#include <qqml.h>

void QtiDynTreeSoleGuiPluginPlugin::registerTypes(const char *uri)
{
    // @uri robotology.idyntree.solegui
    qmlRegisterType<QtiDynTreeSoleGuiPlugin>(uri, 1, 0, "QtiDynTreeSoleGuiPlugin");
}


