#ifndef IDYNTREE_PLOTTER_CHARTSMANAGER_H
#define IDYNTREE_PLOTTER_CHARTSMANAGER_H


#include "thrifts/ChartsService.h"
#include <yarp/os/Port.h>

#include <memory>
#include <mutex>
#include <unordered_map>

class QMainWindow;


class ChartsManager
: public ChartsService
{
    yarp::os::Port m_rpcPort;

    std::mutex m_chartsMutex;
    typedef std::unordered_map<std::string, std::shared_ptr<QMainWindow>> ChartsMap;
    ChartsMap m_charts;

public:

    virtual ~ChartsManager();

    bool initialize();
    void close();

    virtual std::string plotBatchChart(const Chart& chart);
    virtual void closeAllCharts();
    virtual void closeChart(const std::string& chartID);
    virtual std::vector<ChartSimpleInfo> listAllCharts();

};

#endif /* end of include guard: IDYNTREE_PLOTTER_CHARTSMANAGER_H */
