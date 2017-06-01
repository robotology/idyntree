#ifndef IDYNTREE_PLOTTER_CHARTSMANAGERWINDOW_H
#define IDYNTREE_PLOTTER_CHARTSMANAGERWINDOW_H

#include <QMainWindow>

class ChartsManagerWindow : public QMainWindow
{


public:
    ChartsManagerWindow();
    virtual ~ChartsManagerWindow();

    virtual bool event(QEvent *e);
};

#endif /* end of include guard: IDYNTREE_PLOTTER_CHARTSMANAGERWINDOW_H */
