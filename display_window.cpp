#include "display_window.h"

#include <QTabWidget>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <cstdlib>
#include <chrono>

struct MyEdgeInfo : public IEdgeInfo
{
    double weight;
    MyEdgeInfo(double w) : weight(w) {}
    double GetWeight() const override { return weight; }
};

DisplayWindow::DisplayWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setupUi();
}

DisplayWindow::~DisplayWindow()
{
}

void DisplayWindow::setupUi()
{
    auto *tabWidget = new QTabWidget(this);
    setCentralWidget(tabWidget);

    setupTestingTab();
    tabWidget->addTab(testingTab, "Testing");

    setupGraphTab();
    tabWidget->addTab(graphTab, "Performance Graph");

    setupGraphManipulationTab();
    tabWidget->addTab(graphManipTab, "Graph Manipulation");

    resize(800, 600);
    setWindowTitle("Display Window - Example");
}

void DisplayWindow::setupTestingTab()
{
    testingTab = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(testingTab);

    runTestsButton = new QPushButton("Run Tests", testingTab);
    connect(runTestsButton, &QPushButton::clicked, this, &DisplayWindow::runAllTests);

    testOutput = new QTextEdit(testingTab);
    testOutput->setReadOnly(true);

    layout->addWidget(runTestsButton);
    layout->addWidget(new QLabel("Test Output:"));
    layout->addWidget(testOutput);

    testingTab->setLayout(layout);
}

void DisplayWindow::runAllTests()
{
    testOutput->clear();

    std::ofstream file("test_output.txt");
    if (!file.is_open())
    {
        testOutput->append("Failed to open test_output.txt for writing.");
        return;
    }

    auto oldBuf = std::cout.rdbuf(file.rdbuf());

    runTests();
    std::cout.rdbuf(oldBuf);

    file.close();

    QFile qfile("test_output.txt");
    if (qfile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QTextStream in(&qfile);
        testOutput->append(in.readAll());
        qfile.close();
    }
    else
    {
        testOutput->append("Failed to open test_output.txt for reading.");
    }
}

void DisplayWindow::setupGraphTab()
{
    graphTab = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(graphTab);

    chartView = new QtCharts::QChartView(graphTab);
    chartView->setRenderHint(QPainter::Antialiasing);

    QPushButton *loadCsvButton = new QPushButton("Load CSV", graphTab);
    connect(loadCsvButton, &QPushButton::clicked, this, &DisplayWindow::loadCsvData);

    layout->addWidget(chartView);
    layout->addWidget(loadCsvButton);
    graphTab->setLayout(layout);
}

void DisplayWindow::loadCsvData()
{
    QString fileName = QFileDialog::getOpenFileName(
        this, "Open CSV file", QString(), "CSV Files (*.csv);;All files (*.*)");
    if (fileName.isEmpty())
        return;

    csvData.clear();
    csvHeaders.clear();

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Cannot open CSV file:" << fileName;
        return;
    }

    QTextStream in(&file);
    bool firstLine = true;
    while (!in.atEnd())
    {
        QString line = in.readLine().trimmed();
        if (line.isEmpty())
            continue;

        QStringList fields = line.split(",");
        if (fields.isEmpty())
            continue;

        if (firstLine)
        {
            csvHeaders = fields;
            firstLine = false;
        }
        else
        {
            csvData.push_back(fields);
        }
    }
    file.close();

    buildPerformanceChart();
}

void DisplayWindow::buildPerformanceChart()
{
    if (csvData.isEmpty() || csvHeaders.size() < 4)
    {
        qDebug() << "Not enough data in CSV or columns < 4";
        return;
    }

    QtCharts::QChart *chart = new QtCharts::QChart();
    chart->setTitle("Performance: X=Vertices, Y=Time (ms)");

    auto *seriesDijkstra = new QtCharts::QLineSeries();
    seriesDijkstra->setName("Dijkstra");

    auto *seriesBellman = new QtCharts::QLineSeries();
    seriesBellman->setName("Bellman-Ford");

    int colVertices = 0;
    int colDijkstra = 2;
    int colBellman = 3;

    for (auto &row : csvData)
    {
        if (row.size() <= colBellman)
            continue;

        bool okX = false, okD = false, okB = false;
        double xVal = row.at(colVertices).toDouble(&okX);
        double dVal = row.at(colDijkstra).toDouble(&okD);
        double bVal = row.at(colBellman).toDouble(&okB);

        if (okX && okD && okB)
        {
            seriesDijkstra->append(xVal, dVal);
            seriesBellman->append(xVal, bVal);
        }
    }

    chart->addSeries(seriesDijkstra);
    chart->addSeries(seriesBellman);

    auto *axisX = new QtCharts::QValueAxis();
    axisX->setTitleText("Vertices");
    axisX->setLabelFormat("%.0f");

    auto *axisY = new QtCharts::QValueAxis();
    axisY->setTitleText("Time (ms)");
    axisY->setLabelFormat("%.4f");

    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);

    seriesDijkstra->attachAxis(axisX);
    seriesDijkstra->attachAxis(axisY);

    seriesBellman->attachAxis(axisX);
    seriesBellman->attachAxis(axisY);

    chart->legend()->setVisible(true);

    chartView->setChart(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
}

void DisplayWindow::setupGraphManipulationTab()
{
    graphManipTab = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(graphManipTab);

    QHBoxLayout *vertexLayout = new QHBoxLayout();
    vertexLineEdit = new QLineEdit();
    addVertexButton = new QPushButton("Add Vertex");
    removeVertexButton = new QPushButton("Remove Vertex");
    vertexLayout->addWidget(new QLabel("Vertex:"));
    vertexLayout->addWidget(vertexLineEdit);
    vertexLayout->addWidget(addVertexButton);
    vertexLayout->addWidget(removeVertexButton);

    connect(addVertexButton, &QPushButton::clicked, this, &DisplayWindow::addVertex);
    connect(removeVertexButton, &QPushButton::clicked, this, &DisplayWindow::removeVertex);

    QHBoxLayout *edgeLayout = new QHBoxLayout();
    edgeFromLineEdit = new QLineEdit();
    edgeToLineEdit = new QLineEdit();
    addEdgeButton = new QPushButton("Add Edge");
    removeEdgeButton = new QPushButton("Remove Edge");
    edgeLayout->addWidget(new QLabel("From:"));
    edgeLayout->addWidget(edgeFromLineEdit);
    edgeLayout->addWidget(new QLabel("To:"));
    edgeLayout->addWidget(edgeToLineEdit);
    edgeLayout->addWidget(addEdgeButton);
    edgeLayout->addWidget(removeEdgeButton);

    connect(addEdgeButton, &QPushButton::clicked, this, &DisplayWindow::addEdge);
    connect(removeEdgeButton, &QPushButton::clicked, this, &DisplayWindow::removeEdge);

    QHBoxLayout *edgeIdLayout = new QHBoxLayout();
    edgeIdLineEdit = new QLineEdit();
    removeEdgeByIdButton = new QPushButton("Remove Edge by ID");
    edgeIdLayout->addWidget(new QLabel("EdgeID:"));
    edgeIdLayout->addWidget(edgeIdLineEdit);
    edgeIdLayout->addWidget(removeEdgeByIdButton);

    connect(removeEdgeByIdButton, &QPushButton::clicked, this, &DisplayWindow::removeEdgeById);

    QHBoxLayout *randomLayout = new QHBoxLayout();
    randVerticesCount = new QSpinBox();
    randEdgesCount = new QSpinBox();
    randWeightMin = new QDoubleSpinBox();
    randWeightMax = new QDoubleSpinBox();
    randomAddButton = new QPushButton("Random Add");

    randVerticesCount->setRange(0, 1000);
    randEdgesCount->setRange(0, 5000);
    randWeightMin->setRange(0, 99999);
    randWeightMax->setRange(0, 99999);
    randVerticesCount->setValue(5);
    randEdgesCount->setValue(10);
    randWeightMin->setValue(1.0);
    randWeightMax->setValue(10.0);

    connect(randomAddButton, &QPushButton::clicked, this, &DisplayWindow::randomAdd);

    randomLayout->addWidget(new QLabel("Add Vertices:"));
    randomLayout->addWidget(randVerticesCount);
    randomLayout->addWidget(new QLabel("Add Edges:"));
    randomLayout->addWidget(randEdgesCount);
    randomLayout->addWidget(new QLabel("WeightMin:"));
    randomLayout->addWidget(randWeightMin);
    randomLayout->addWidget(new QLabel("Max:"));
    randomLayout->addWidget(randWeightMax);
    randomLayout->addWidget(randomAddButton);

    QHBoxLayout *dijkstraLayout = new QHBoxLayout();
    dijkstraStart = new QLineEdit();
    dijkstraEnd = new QLineEdit();
    dijkstraButton = new QPushButton("Dijkstra");
    connect(dijkstraButton, &QPushButton::clicked, this, &DisplayWindow::runDijkstra);

    dijkstraLayout->addWidget(new QLabel("Start:"));
    dijkstraLayout->addWidget(dijkstraStart);
    dijkstraLayout->addWidget(new QLabel("End:"));
    dijkstraLayout->addWidget(dijkstraEnd);
    dijkstraLayout->addWidget(dijkstraButton);

    QHBoxLayout *exportLayout = new QHBoxLayout();
    exportDotButton = new QPushButton("Export to DOT");
    renderButton = new QPushButton("Render Graph");
    connect(exportDotButton, &QPushButton::clicked, this, &DisplayWindow::exportGraphToDot);
    connect(renderButton, &QPushButton::clicked, this, &DisplayWindow::renderGraphImage);

    exportLayout->addWidget(exportDotButton);
    exportLayout->addWidget(renderButton);

    mainLayout->addLayout(vertexLayout);
    mainLayout->addLayout(edgeLayout);
    mainLayout->addLayout(edgeIdLayout);
    mainLayout->addLayout(randomLayout);
    mainLayout->addLayout(dijkstraLayout);
    mainLayout->addLayout(exportLayout);

    graphView = new GraphicsViewZoom();
    graphScene = new QGraphicsScene(graphManipTab);
    graphView->setScene(graphScene);
    graphView->setRenderHint(QPainter::Antialiasing);
    graphView->setDragMode(QGraphicsView::ScrollHandDrag);

    mainLayout->addWidget(graphView);

    graphManipTab->setLayout(mainLayout);
}

void DisplayWindow::addVertex()
{
    bool ok;
    int v = vertexLineEdit->text().toInt(&ok);
    if (!ok)
    {
        QMessageBox::warning(this, "Error", "Invalid vertex ID");
        return;
    }
    myGraph.AddVertex(v);
    QMessageBox::information(this, "OK", QString("Vertex %1 added").arg(v));
}

void DisplayWindow::removeVertex()
{
    bool ok;
    int v = vertexLineEdit->text().toInt(&ok);
    if (!ok)
    {
        QMessageBox::warning(this, "Error", "Invalid vertex ID");
        return;
    }
    try
    {
        myGraph.RemoveVertex(v);
        QMessageBox::information(this, "OK", QString("Vertex %1 removed").arg(v));
    }
    catch (const std::exception &e)
    {
        QMessageBox::warning(this, "Error", e.what());
    }
}

void DisplayWindow::addEdge()
{
    bool ok1, ok2;
    int f = edgeFromLineEdit->text().toInt(&ok1);
    int t = edgeToLineEdit->text().toInt(&ok2);
    if (!ok1 || !ok2)
    {
        QMessageBox::warning(this, "Error", "Invalid from/to");
        return;
    }

    EdgeProperties data(-1, ShrdPtr<IEdgeInfo>(new MyEdgeInfo(1.0)));
    myGraph.AddEdge(f, t, data);
    QMessageBox::information(this, "OK", QString("Edge %1->%2 added").arg(f).arg(t));
}

void DisplayWindow::removeEdge()
{
    bool ok1, ok2;
    int f = edgeFromLineEdit->text().toInt(&ok1);
    int t = edgeToLineEdit->text().toInt(&ok2);
    if (!ok1 || !ok2)
    {
        QMessageBox::warning(this, "Error", "Invalid from/to");
        return;
    }
    try
    {
        myGraph.RemoveEdge(f, t);
        QMessageBox::information(this, "OK", QString("Edge %1->%2 removed").arg(f).arg(t));
    }
    catch (const std::exception &e)
    {
        QMessageBox::warning(this, "Error", e.what());
    }
}

void DisplayWindow::removeEdgeById()
{
    bool ok1, ok2, ok3;
    int f = edgeFromLineEdit->text().toInt(&ok1);
    int t = edgeToLineEdit->text().toInt(&ok2);
    int id = edgeIdLineEdit->text().toInt(&ok3);
    if (!ok1 || !ok2 || !ok3)
    {
        QMessageBox::warning(this, "Error", "Invalid from/to/ID");
        return;
    }
    myGraph.RemoveEdgeById(f, t, id);
    QMessageBox::information(this, "OK", QString("Edge ID=%1 from %2->%3 removed").arg(id).arg(f).arg(t));
}

void DisplayWindow::randomAdd()
{
    int nv = randVerticesCount->value();
    int ne = randEdgesCount->value();
    double wMin = randWeightMin->value();
    double wMax = randWeightMax->value();

    if (wMin > wMax)
    {
        QMessageBox::warning(this, "Error", "Weight min > max!");
        return;
    }

    for (int i = 0; i < nv; i++)
    {
        int v = rand() % 10000;
        myGraph.AddVertex(v);
    }
    auto verts = myGraph.GetVertices();
    int vCount = verts.GetLength();
    if (vCount < 2)
    {
        QMessageBox::information(this, "Info", "Not enough vertices to create edges");
        return;
    }
    for (int i = 0; i < ne; i++)
    {
        int fromIdx = rand() % vCount;
        int toIdx = rand() % vCount;
        int fromV = verts.Get(fromIdx);
        int toV = verts.Get(toIdx);

        double w = wMin + (double(rand()) / RAND_MAX) * (wMax - wMin);
        EdgeProperties data(-1, ShrdPtr<IEdgeInfo>(new MyEdgeInfo(w)));
        myGraph.AddEdge(fromV, toV, data);
    }
    QMessageBox::information(this, "OK", QString("Added %1 vertices, %2 edges randomly").arg(nv).arg(ne));
}

void DisplayWindow::runDijkstra()
{
    bool ok1, ok2;
    int start = dijkstraStart->text().toInt(&ok1);
    int end = dijkstraEnd->text().toInt(&ok2);
    if (!ok1 || !ok2)
    {
        QMessageBox::warning(this, "Error", "Invalid start/end vertex");
        return;
    }

    ClearShortestPath();

    DijkstraAlgorithm<int> dAlg;
    auto result = dAlg.FindPath(myGraph, start, end);
    if (result.distance == std::numeric_limits<double>::infinity())
    {
        QMessageBox::information(this, "No Path", "No path found (distance=Infinity).");
    }
    else
    {
        MarkShortestPath(result.path);
        QMessageBox::information(this, "Path found",
                                 QString("Distance = %1\nPath length = %2 edges")
                                     .arg(result.distance)
                                     .arg(result.path.GetLength()));
    }
    renderGraphImage();
}

void DisplayWindow::ClearShortestPath()
{
    pathEdgeIds.clear();
    pathVertices.clear();
}

void DisplayWindow::MarkShortestPath(const DynamicArraySmart<PathEdge<int>> &pathEdges)
{
    for (int i = 0; i < pathEdges.GetLength(); i++)
    {
        const auto &pe = pathEdges.Get(i);

        pathVertices.insert(pe.from);
        pathVertices.insert(pe.to);

        pathEdgeIds.insert(pe.edgeData.id);
    }
}

void DisplayWindow::exportGraphToDot()
{
    QString fileName = QFileDialog::getSaveFileName(this, "Save DOT file", "", "DOT Files (*.dot)");
    if (fileName.isEmpty())
        return;

    std::ofstream ofs(fileName.toStdString());
    if (!ofs.is_open())
    {
        QMessageBox::warning(this, "Error", "Cannot open dot file for writing");
        return;
    }

    if (myGraph.IsDirected())
    {
        ofs << "digraph G {\n"
            << "  graph [pad=\"1.0\", ranksep=\"10.0\", nodesep=\"10.0\", size=\"10,10\", dpi=300];\n"
            << "  node [shape=circle, style=filled, fillcolor=white];\n"
            << "  edge [color=black];\n";
    }
    else
    {
        ofs << "graph G {\n"
            << "  graph [layout=fdp, overlap=false, splines=true, sep=\"+150\", nodesep=\"10.0\", ranksep=\"10.0\", size=\"10,10\", dpi=300];\n"
            << "  node [shape=circle, style=filled, fillcolor=white];\n"
            << "  edge [color=black];\n";
    }

    auto vertices = myGraph.GetVertices();
    for (int i = 0; i < vertices.GetLength(); i++)
    {
        int v = vertices.Get(i);

        if (pathVertices.find(v) != pathVertices.end())
        {
            ofs << "  \"" << v << "\" [fillcolor=\"lime\"];\n";
        }
        else
        {
            ofs << "  \"" << v << "\";\n";
        }
    }
    auto allEdges = myGraph.GetEdges();
    for (int i = 0; i < allEdges.GetLength(); i++)
    {
        auto &gEdge = allEdges.Get(i);
        int from = gEdge.from;
        int to   = gEdge.to;

        auto &edgeArr = gEdge.edges;
        for (int k = 0; k < edgeArr.GetLength(); k++)
        {
            auto &ed = edgeArr.Get(k);

            double w = (ed.info) ? ed.info->GetWeight() : 0.0;
            bool inPath = (pathEdgeIds.find(ed.id) != pathEdgeIds.end());

            std::string lbl = "id=" + std::to_string(ed.id) + ", w=" + std::to_string(w);

            std::string extras;
            if (inPath)
            {
                extras += ", color=red, penwidth=2.0";
            }

            if (myGraph.IsDirected())
            {
                ofs << "  \"" << from << "\" -> \"" << to << "\" [label=\"" << lbl << "\"" << extras << "];\n";
            }
            else
            {
                ofs << "  \"" << from << "\" -- \"" << to << "\" [label=\"" << lbl << "\"" << extras << "];\n";
            }
        }
    }

    ofs << "}\n";
    ofs.close();

    QMessageBox::information(this, "Exported", "Graph exported to DOT.");
}

void DisplayWindow::renderGraphImage()
{
    std::string dotFile = "temp_graph.dot";
    {
        std::ofstream ofs(dotFile);
        if (!ofs.is_open())
        {
            QMessageBox::warning(this, "Error", "Cannot open temp_graph.dot");
            return;
        }
        if (myGraph.IsDirected())
        {
            ofs << "digraph G {\n"
                << "  graph [pad=\"1.0\", ranksep=\"10.0\", nodesep=\"10.0\", size=\"10,10\", dpi=300];\n"
                << "  node [shape=circle, style=filled, fillcolor=white];\n"
                << "  edge [color=black];\n";
        }
        else
        {
            ofs << "graph G {\n"
                << "  graph [layout=fdp, overlap=false, splines=true, sep=\"+150\", nodesep=\"10.0\", ranksep=\"10.0\", size=\"10,10\", dpi=300];\n"
                << "  node [shape=circle, style=filled, fillcolor=white];\n"
                << "  edge [color=black];\n";
        }

        auto vertices = myGraph.GetVertices();
        for (int i = 0; i < vertices.GetLength(); i++)
        {
            int v = vertices.Get(i);

            if (pathVertices.find(v) != pathVertices.end())
            {
                ofs << "  \"" << v << "\" [fillcolor=\"lime\"];\n";
            }
            else
            {
                ofs << "  \"" << v << "\";\n";
            }
        }

        auto allEdges = myGraph.GetEdges();
        for (int i = 0; i < allEdges.GetLength(); i++)
        {
            auto &gEdge = allEdges.Get(i);
            int from = gEdge.from;
            int to   = gEdge.to;
            auto &edgeArr = gEdge.edges;

            for (int k = 0; k < edgeArr.GetLength(); k++)
            {
                auto &ed = edgeArr.Get(k);

                double w = (ed.info) ? ed.info->GetWeight() : 0.0;
                bool inPath = (pathEdgeIds.find(ed.id) != pathEdgeIds.end());

                std::string lbl = "id=" + std::to_string(ed.id) + ", w=" + std::to_string(w);

                std::string extras;
                if (inPath)
                {
                    extras += ", color=red, penwidth=2.0";
                }

                if (myGraph.IsDirected())
                {
                    ofs << "  \"" << from << "\" -> \"" << to << "\" [label=\"" << lbl << "\"" << extras << "];\n";
                }
                else
                {
                    ofs << "  \"" << from << "\" -- \"" << to << "\" [label=\"" << lbl << "\"" << extras << "];\n";
                }
            }
        }

        ofs << "}\n";
    }

    std::string pngFile = "temp_graph.png";
    QString cmd;
    if (myGraph.IsDirected())
    {
        cmd = QString("dot -Tpng \"%1\" -o \"%2\"").arg(QString::fromStdString(dotFile),
                                                                QString::fromStdString(pngFile));
    }
    else
    {
        cmd = QString("fdp -Tpng \"%1\" -o \"%2\"").arg(QString::fromStdString(dotFile),
                                                                  QString::fromStdString(pngFile));
    }

    int ret = system(cmd.toUtf8().constData());
    if (ret != 0)
    {
        QMessageBox::warning(this, "Error", "Failed to run Graphviz. Ensure it's installed and in PATH.");
        return;
    }

    QPixmap pix(QString::fromStdString(pngFile));
    if (pix.isNull())
    {
        QMessageBox::warning(this, "Error", "Failed to load temp_graph.png");
        return;
    }

    graphScene->clear();
    pixmapItem = graphScene->addPixmap(pix);
    graphView->fitInView(pixmapItem, Qt::KeepAspectRatio);
}




