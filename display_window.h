#ifndef DISPLAY_WINDOW_H
#define DISPLAY_WINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QTabWidget>
#include <QTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QStringList>
#include <QVector>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <unordered_set>
#include <fstream>
#include <iostream>

#include "graphicsview_zoom.h"
#include "test.h"
#include "DataStructures/Graph.h"
#include "DataStructures/IEdgeInfo.h"
#include "DataStructures/ShortestPathAlgorithms.h"

class DisplayWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit DisplayWindow(QWidget *parent = nullptr);
    ~DisplayWindow();

private slots:

    void runAllTests();

    void loadCsvData();
    void buildPerformanceChart();

    void addVertex();
    void removeVertex();
    void addEdge();
    void removeEdge();
    void removeEdgeById();
    void randomAdd();
    void runDijkstra();
    void exportGraphToDot();
    void renderGraphImage();

private:
    void setupUi();
    void setupTestingTab();
    void setupGraphTab();
    void setupGraphManipulationTab();

    void ClearShortestPath();
    void MarkShortestPath(const DynamicArraySmart<PathEdge<int>> &pathEdges);

    QWidget *testingTab;
    QPushButton *runTestsButton;
    QTextEdit *testOutput;

    QWidget *graphTab;
    QtCharts::QChartView *chartView;

    QStringList csvHeaders;
    QVector<QStringList> csvData;

    QWidget *graphManipTab;

    QLineEdit *vertexLineEdit;
    QPushButton *addVertexButton;
    QPushButton *removeVertexButton;

    QLineEdit *edgeFromLineEdit;
    QLineEdit *edgeToLineEdit;
    QPushButton *addEdgeButton;
    QPushButton *removeEdgeButton;

    QLineEdit *edgeIdLineEdit;
    QPushButton *removeEdgeByIdButton;

    QSpinBox *randVerticesCount;
    QSpinBox *randEdgesCount;
    QDoubleSpinBox *randWeightMin;
    QDoubleSpinBox *randWeightMax;
    QPushButton *randomAddButton;

    QLineEdit *dijkstraStart;
    QLineEdit *dijkstraEnd;
    QPushButton *dijkstraButton;

    QPushButton *exportDotButton;
    QPushButton *renderButton;

    QGraphicsView *graphView;
    QGraphicsScene *graphScene;
    QGraphicsPixmapItem *pixmapItem;

    Graph<int> myGraph;

    std::unordered_set<int> pathEdgeIds;
    std::unordered_set<int> pathVertices;
};

#endif
