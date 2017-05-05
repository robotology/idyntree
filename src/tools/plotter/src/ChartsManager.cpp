
#include "ChartsManager.h"

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QApplication>
#include <QtWidgets/QMainWindow>


#include <cstdlib>
#include <iostream>
#include <sstream>

// equivalent of dispatch_async
//http://stackoverflow.com/questions/21646467/how-to-execute-a-functor-or-a-lambda-in-a-given-thread-in-qt-gcd-style
template <typename F>
static void postToThread(F && fun, QObject * obj = qApp) {
    struct Event : public QEvent {
        using Fun = typename std::decay<F>::type;
        Fun fun;
        Event(Fun && fun) : QEvent(QEvent::None), fun(std::move(fun)) {}
        Event(const Fun & fun) : QEvent(QEvent::None), fun(fun) {}
        ~Event() { fun(); }
    };
    QCoreApplication::postEvent(obj, new Event(std::forward<F>(fun)));
}

ChartsManager::~ChartsManager()
{
    close();
}

bool ChartsManager::initialize()
{
    yarp::os::ResourceFinder& finder = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string serverName = finder.check("name", yarp::os::Value("idyntree-plotter")).asString();

    if (!m_rpcPort.open("/" + serverName + "/rpc")) {
        return false;
    }
    return yarp().attachAsServer(m_rpcPort);
}

void ChartsManager::close()
{
    m_rpcPort.close();

    std::lock_guard<std::mutex> guard(m_chartsMutex);
    m_charts.clear();
}

//TODO: do not work the following:
// Single window instead of multiple windows
// series are empty even if transmitted

std::string ChartsManager::plotBatchChart(const Chart& chart)
{
    //Create a unique key
    std::ostringstream keyStream;
    keyStream << "Chart_" << rand();
    keyStream << "_" << yarp::os::Time::now();
    std::string key = keyStream.str();

    //temporary objects such as chart and key must be copied
    //? I thought they were retained untile the lambda finished execution..
    postToThread([chart, this, key]{
        using namespace QtCharts;

        //Creating a new window for the chart
        QMainWindow *newWindow = new QMainWindow();
        // the following is more for debug
        newWindow->setWindowTitle(key.c_str());

        QChartView * newChart = new QChartView();

        newChart->chart()->setTitle(chart.title.c_str());
        for (auto& chartSeries : chart.series) {
            QLineSeries *series = new QLineSeries();
            for (int index = 0; index < chartSeries.y.size(); ++index) {
                series->append(chartSeries.x(index), chartSeries.y(index));
            }
            series->setName(chartSeries.name.c_str());
            newChart->chart()->addSeries(series);
            newChart->chart()->createDefaultAxes();
        }

        // now assign the chart to the window (the window takes ownership
        newWindow->setCentralWidget(newChart);
//        newChart->chart()->axisX()->setTitleText(chart.xAxisTitle.c_str());
//        newChart->chart()->axisY()->setTitleText(chart.yAxisTitle.c_str());

        newChart->show();
        newWindow->show();

        {
            std::lock_guard<std::mutex> guard(m_chartsMutex);
            m_charts.insert(ChartsMap::value_type(key, std::shared_ptr<QMainWindow>(newWindow)));
        }
    });

    return key;
}

void ChartsManager::closeAllCharts()
{
    postToThread([this]{
        std::lock_guard<std::mutex> guard(m_chartsMutex);
        m_charts.clear();
            });
}

void ChartsManager::closeChart(const std::string& chartID)
{

    postToThread([this,chartID]{
        std::lock_guard<std::mutex> guard(m_chartsMutex);
        ChartsMap::iterator found = m_charts.find(chartID);
        if (found != m_charts.end()) {
            m_charts.erase(found);
        }
    });
}

std::vector<ChartSimpleInfo> ChartsManager::listAllCharts()
{
    std::vector<ChartSimpleInfo> infos;
    std::lock_guard<std::mutex> guard(m_chartsMutex);
    for (const auto &pair : m_charts) {
        ChartSimpleInfo info;
        info.id = pair.first;
        QtCharts::QChartView* chartView = static_cast<QtCharts::QChartView*>(pair.second->centralWidget());
        if (!chartView) continue;
        info.title = chartView->chart()->title().toStdString();
        infos.push_back(info);
    }

    return infos;

}
