#include "ChartsManager.h"

#include "ChartsManagerWindow.h"

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/SystemClock.h>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtWidgets/QMainWindow>
#include <QApplication>
#include <QTimer>
#include <QObject>



#include <cstdlib>
#include <iostream>
#include <mutex>
#include <condition_variable>
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

ChartsManager::ChartInfo::ChartInfo(std::string id)
: m_id(id) {}

bool ChartsManager::ChartInfo::init() { return true; }

ChartsManager::RealTimeChartInfo::RealTimeChartInfo(const std::string& id)
: ChartsManager::ChartInfo(id) {}

bool ChartsManager::RealTimeChartInfo::init()
{
    if (!m_streamingPort.open("/" + id() + "/series:i"))
        return false;
    m_streamingPort.useCallback(*this);

    QTimer *timer = new QTimer(window->centralWidget());
    QObject::connect(timer, &QTimer::timeout, [this](){
        using namespace QtCharts;
        QChartView* chartView = static_cast<QChartView*>(window->centralWidget());
        if (!chartView) return;

        std::lock_guard<std::mutex> guard(m_bufferMutex);
        if (chartSeries.size() != m_buffer.size()) return;

        for (size_t index = 0; index < m_buffer.size(); ++index) {
            QXYSeries* series = static_cast<QXYSeries*>(chartSeries[index]);
            yarp::sig::Vector &x = m_buffer[index].first;
            yarp::sig::Vector &y = m_buffer[index].second;
            for (int point = 0; point < x.size() ; ++point) {
                series->append(x(point), y(point));
                std::cerr << "Adding " << x(point) << " " << y(point) << "\n";
            }
            x.resize(0);
            y.resize(0);

            if (m_xAxis.axisType == ChartInfo::Axis::AxisTypeDynamic) {
                // Update values so as to stay in the range

                QAbstractAxis* xaxis = chartView->chart()->axisX(series);
                if (xaxis && series->count() > 0) {
                    unsigned startingIndex = 0;

                    while (series->at(series->count() - 1).x() - series->at(startingIndex).x() > (m_xAxis.max - m_xAxis.min)) {
                        startingIndex++;
                    }
                    //if (startingIndex > 0) {
                        series->removePoints(0, startingIndex);
                        //Now updating the mix/max value for the new range
                        if (m_xAxis.axisType == ChartInfo::Axis::AxisTypeDynamic) {
                            xaxis->setRange(QVariant(series->at(0).x()),
                                            QVariant(series->at(series->count() - 1).x()));
                        }
                    //}
                }
            }
        }

    });
    timer->start(250);

    return true;
}

ChartsManager::RealTimeChartInfo::~RealTimeChartInfo()
{
    m_streamingPort.close();
}

void ChartsManager::RealTimeChartInfo::onRead(RealTimeStreamData &datum)
{
    std::lock_guard<std::mutex> guard(m_bufferMutex);
    m_buffer.resize(datum.newPoints.size());
    for (size_t index = 0; index < datum.newPoints.size(); ++index) {
        const yarp::sig::Vector &newX = datum.newPoints[index].x;
        const yarp::sig::Vector &newY = datum.newPoints[index].y;
        yarp::sig::Vector &xBuffer = m_buffer[index].first;
        yarp::sig::Vector &yBuffer = m_buffer[index].second;

        int lastElement = xBuffer.size();
        xBuffer.resize(lastElement + newX.size());
        yBuffer.resize(lastElement + newY.size());
        for (int point = 0; point < newX.size() ; ++point, ++lastElement) {
            xBuffer(lastElement) = newX(point);
            yBuffer(lastElement) = newY(point);
        }
    }
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
    bool result = yarp().attachAsServer(m_rpcPort);

    // Create main window
    m_mainWindow = std::unique_ptr<ChartsManagerWindow>(new ChartsManagerWindow());
    if (!m_mainWindow) {
        close();
        return false;
    }
    m_mainWindow->show();

    return result;
}

void ChartsManager::close()
{
    m_rpcPort.close();

    std::lock_guard<std::mutex> guard(m_chartsMutex);
    m_charts.clear();
}

bool ChartsManager::initializeChart(const Chart& chart,std::shared_ptr<ChartsManager::ChartInfo> chartInfo)
{
    using namespace QtCharts;
    //Creating a new window for the chart
    QMainWindow *newWindow = new QMainWindow();
    chartInfo->window = std::shared_ptr<QMainWindow>(newWindow);

    QChartView * newChart = new QChartView();

    newChart->chart()->setTitle(chart.title.c_str());
    chartInfo->chartSeries.reserve(chart.series.size());

    for (auto& chartSeries : chart.series) {
        QLineSeries *series = new QLineSeries();
        chartInfo->chartSeries.push_back(series);

        for (int index = 0; index < chartSeries.y.size(); ++index) {
            series->append(chartSeries.x(index), chartSeries.y(index));
        }
        series->setName(chartSeries.name.c_str());
        newChart->chart()->addSeries(series);
    }

    // Create default axes after all series have been added
    newChart->chart()->createDefaultAxes();

    //set axes
    chartInfo->m_xAxis.min = chart.xAxis.minimum;
    chartInfo->m_xAxis.max = chart.xAxis.maximum;

    if (chart.xAxis.type == "auto")
        chartInfo->m_xAxis.axisType = ChartInfo::Axis::AxisTypeAuto;
    else if (chart.xAxis.type == "dyn") {
        chartInfo->m_xAxis.axisType = ChartInfo::Axis::AxisTypeDynamic;
    } else {
        chartInfo->m_xAxis.axisType = ChartInfo::Axis::AxisTypeFixed;
        QVariant min = QVariant(chartInfo->m_xAxis.min);
        QVariant max = QVariant(chartInfo->m_xAxis.max);
        newChart->chart()->axisX()->setRange(min, max);
    }

    chartInfo->m_yAxis.min = chart.yAxis.minimum;
    chartInfo->m_yAxis.max = chart.yAxis.maximum;
    if (chart.yAxis.type == "auto")
        chartInfo->m_yAxis.axisType = ChartInfo::Axis::AxisTypeAuto;
    else if (chart.yAxis.type == "dyn") {
        chartInfo->m_yAxis.axisType = ChartInfo::Axis::AxisTypeDynamic;
    } else {
        chartInfo->m_yAxis.axisType = ChartInfo::Axis::AxisTypeFixed;
        QVariant min = QVariant(chartInfo->m_yAxis.min);
        QVariant max = QVariant(chartInfo->m_yAxis.max);
        newChart->chart()->axisY()->setRange(min, max);
    }

    newChart->chart()->axisX()->setTitleText(chart.xAxis.title.c_str());
    newChart->chart()->axisY()->setTitleText(chart.yAxis.title.c_str());


    // now assign the chart to the window (the window takes ownership
    newWindow->setCentralWidget(newChart);


    newChart->show();
    newWindow->show();
    return true;
}

std::shared_ptr<ChartsManager::ChartInfo> ChartsManager::createNewBatchChart(const Chart& chart,
                                               const std::string& id)
{
    std::shared_ptr<ChartInfo> chartInfo = std::shared_ptr<ChartInfo>(new ChartInfo(id));
    initializeChart(chart, chartInfo);
    return chartInfo;
}

std::shared_ptr<ChartsManager::RealTimeChartInfo> ChartsManager::createNewRealTimeChart(const Chart& chart,
                                                  const std::string& id,
                                                                                const RealTimeChartRequest& chartAddInfo)
{
    std::shared_ptr<RealTimeChartInfo> chartInfo = std::shared_ptr<RealTimeChartInfo>(new RealTimeChartInfo(id));
    initializeChart(chart, chartInfo);

    //get x axis
    using namespace QtCharts;
    QChartView *chartView = static_cast<QChartView*>(chartInfo->window->centralWidget());
    if (!chartView) return chartInfo;

    for (auto& series : chartInfo->chartSeries) {
        QAbstractAxis* xaxis = chartView->chart()->axisX(series);
        if (!xaxis) {
            xaxis = new QValueAxis();
            chartView->chart()->setAxisX(xaxis);
        }

        if (chartInfo->m_xAxis.axisType != ChartInfo::Axis::AxisTypeAuto) {
            QVariant min = QVariant(chartInfo->m_xAxis.min);
            QVariant max = QVariant(chartInfo->m_xAxis.max);
            if (chartInfo->m_xAxis.axisType == ChartInfo::Axis::AxisTypeDynamic) {
                max = QVariant(chartInfo->m_xAxis.min + chartInfo->m_xAxis.max);
            }
            xaxis->setRange(min, max);
        }

        QAbstractAxis* yaxis = chartView->chart()->axisY(series);
        if (!yaxis) {
            yaxis = new QValueAxis();
            chartView->chart()->setAxisY(yaxis);
        }
        if (chartInfo->m_yAxis.axisType != ChartInfo::Axis::AxisTypeAuto) {
            QVariant min = QVariant(chartInfo->m_yAxis.min);
            QVariant max = QVariant(chartInfo->m_yAxis.max);
            if (chartInfo->m_yAxis.axisType == ChartInfo::Axis::AxisTypeDynamic) {
                max = QVariant(chartInfo->m_yAxis.min + chartInfo->m_yAxis.max);
            }
            yaxis->setRange(min, max);
        }

    }

    return chartInfo;
}

std::string ChartsManager::createChartKey(const Chart& chart)
{
    //Create a unique key
    std::ostringstream keyStream;
    keyStream << "Chart_" << rand();
    keyStream << "_" << yarp::os::SystemClock::nowSystem();
    return keyStream.str();

}

std::string ChartsManager::plotBatchChart(const Chart& chart)
{
    std::string key = createChartKey(chart);

    //temporary objects such as chart and key must be copied
    postToThread([chart, this, key]{
        std::shared_ptr<ChartInfo> chartInfo = this->createNewBatchChart(chart, key);
        // the following is more for debug
        chartInfo->window->setWindowTitle(key.c_str());
        {
            std::lock_guard<std::mutex> guard(m_chartsMutex);
            m_charts.insert(ChartsMap::value_type(key, chartInfo));
        }
    });

    return key;
}

RealTimeChartResponse ChartsManager::plotRealTimeChart(const Chart& chart, const RealTimeChartRequest& chartAddInfo)
{
    RealTimeChartResponse response;
    std::string key = createChartKey(chart);

    std::mutex synchMutex;
    bool methodDone = false;
    std::condition_variable synchVariable;
    std::unique_lock<std::mutex> lock(synchMutex);

    //temporary objects such as chart and key must be copied
    postToThread([chart, this, key, chartAddInfo,
                  &synchMutex, &synchVariable, &methodDone]{
        std::shared_ptr<ChartInfo> chartInfo = this->createNewRealTimeChart(chart, key, chartAddInfo);
        chartInfo->init();
        //customize the chart

        // the following is more for debug
        chartInfo->window->setWindowTitle(key.c_str());
        {
            std::lock_guard<std::mutex> guard(m_chartsMutex);
            m_charts.insert(ChartsMap::value_type(key, chartInfo));
        }

        std::unique_lock<std::mutex> lock(synchMutex);
        methodDone = true;
        synchVariable.notify_one();
    });

    synchVariable.wait(lock, [&methodDone]() { return methodDone; });

    std::lock_guard<std::mutex> guard(m_chartsMutex);
    ChartsMap::const_iterator newChart = m_charts.find(key);
    if (newChart != m_charts.end()) {
        response.id = key;
        response.streamingPortName = static_cast<RealTimeChartInfo*>(newChart->second.get())->portName();
    }


    return response;
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
        QtCharts::QChartView* chartView = static_cast<QtCharts::QChartView*>(pair.second->window->centralWidget());
        if (!chartView) continue;
        info.title = chartView->chart()->title().toStdString();
        infos.push_back(info);
    }

    return infos;

}
