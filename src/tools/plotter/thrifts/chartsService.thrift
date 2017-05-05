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

struct Chart {
    1: string title;
    2: string xAxisTitle;
    3: string yAxisTitle;
    4: list<Series> series;
}

struct ChartSimpleInfo {
    1: string id;
    2: string title;
}

service ChartsService {
    string plotBatchChart(1: Chart chart);
    oneway void closeAllCharts();
    oneway void closeChart(1: string chartID);

    list<ChartSimpleInfo> listAllCharts();
}

