#ifndef IDYNTREE_PLOTTER_CHARTSMANAGER_H
#define IDYNTREE_PLOTTER_CHARTSMANAGER_H


#include <ChartsService.h>
#include <RealTimeStreamData.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

class QMainWindow;
namespace QtCharts {
    class QAbstractSeries;
}

#include "ChartsManagerWindow.h" //to be removed 
class ChartsManagerWindow;

class ChartsManager
: public ChartsService
{
    //Hierarchy of classes
    class ChartInfo {
    public:

        struct Axis {
            enum AxisType {
                AxisTypeFixed,
                AxisTypeAuto,
                AxisTypeDynamic,
            } axisType;

            double min;
            double max;
        };

    private:

        std::string m_id;

        //for now support only one axis for all the chart (not for series)
        Axis m_xAxis;
        Axis m_yAxis;

    public:

        ChartInfo(std::string id);

        inline const std::string& id() const { return m_id; }
        std::shared_ptr<QMainWindow> window;
        std::vector<QtCharts::QAbstractSeries*> chartSeries;

        virtual bool init();

        friend class ChartsManager;
    };

    class RealTimeChartInfo
    : public ChartInfo
    , public yarp::os::TypedReaderCallback<RealTimeStreamData> {
        yarp::os::BufferedPort<RealTimeStreamData> m_streamingPort;
        std::vector<std::pair<yarp::sig::Vector, yarp::sig::Vector>> m_buffer;
        std::mutex m_bufferMutex;

    public:
        RealTimeChartInfo(const std::string& id);
        virtual ~RealTimeChartInfo();

        inline std::string portName() const { return m_streamingPort.getName(); }
        virtual bool init();

        virtual void onRead (RealTimeStreamData& datum);

        friend class ChartsManager;

    };


    //Main window
    std::unique_ptr<ChartsManagerWindow> m_mainWindow;

    yarp::os::Port m_rpcPort;

    std::mutex m_chartsMutex;
    typedef std::unordered_map<std::string, std::shared_ptr<ChartInfo>> ChartsMap;
    ChartsMap m_charts;


    //Must be called on main thread
    bool initializeChart(const Chart& chart, std::shared_ptr<ChartInfo> chartInfo);
    std::shared_ptr<ChartInfo> createNewBatchChart(const Chart& chart,
                                                   const std::string& id);
    std::shared_ptr<RealTimeChartInfo> createNewRealTimeChart(const Chart& chart,
                                                      const std::string& id,
                                                      const RealTimeChartRequest& chartAddInfo);


    std::string createChartKey(const Chart& chart);
public:

    virtual ~ChartsManager();

    bool initialize();
    void close();

    virtual std::string plotBatchChart(const Chart& chart);
    virtual RealTimeChartResponse plotRealTimeChart(const Chart& chart, const RealTimeChartRequest& chartInfo);
//    virtual bool realTimeChartAddValue(const std::string& chartID, const std::vector<Series> & additionaPoints);

    virtual void closeAllCharts();
    virtual void closeChart(const std::string& chartID);
    virtual std::vector<ChartSimpleInfo> listAllCharts();

};

#endif /* end of include guard: IDYNTREE_PLOTTER_CHARTSMANAGER_H */
