struct Vector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

struct Series {
    1: string name;
    2: Vector x;
    3: Vector y;
}

struct AxisInfo {
    1: string type = "auto";
    2: double minimum = 0;
    3: double maximum = 30;
    4: string title;
}

struct Chart {
    1: string title;
    2: AxisInfo xAxis;
    3: AxisInfo yAxis;
    4: list<Series> series;
}

struct RealTimeChartRequest {
    1: bool dummy;
}

struct RealTimeChartResponse {
    1: string id;
    2: string streamingPortName;
}

struct ChartSimpleInfo {
    1: string id;
    2: string title;
}

struct RealTimeStreamData {
    1: list<Series> newPoints;
}


service ChartsService {
    string plotBatchChart(1: Chart chart);
    RealTimeChartResponse plotRealTimeChart(1: Chart chart, 2: RealTimeChartRequest chartInfo);

    oneway void closeAllCharts();
    oneway void closeChart(1: string chartID);

    list<ChartSimpleInfo> listAllCharts();
}

