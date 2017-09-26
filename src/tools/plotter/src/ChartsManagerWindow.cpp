#include "ChartsManagerWindow.h"

#include <QListWidget>
#include <QEvent>

#include <iostream>
ChartsManagerWindow::ChartsManagerWindow()
{
    QListWidget* listView = new QListWidget (this);
    this->setCentralWidget(listView);
    
}


ChartsManagerWindow::~ChartsManagerWindow() {}


bool ChartsManagerWindow::event(QEvent *e)
{
//    std::cerr << "Event:" << e->type() << "\n";
    if (e->type() == QEvent::ActivationChange && !isVisible()) {
//        show();
//        std::cerr << "Show!!\n";
    }
    return QMainWindow::event(e);
}
