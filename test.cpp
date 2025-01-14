#include "test.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <cstdlib>
#include <vector>

#include "DataStructures/Graph.h"
#include "DataStructures/ShortestPathAlgorithms.h"
#include "DataStructures/IEdgeInfo.h"

struct MyEdgeInfo : public IEdgeInfo
{
    double weight;
    MyEdgeInfo(double w) : weight(w) {}
    double GetWeight() const override
    {
        return weight;
    }
};

struct GraphParams
{
    int numVertices;
    int numEdges;
};

static std::vector<GraphParams> LoadAllConfigs(const std::string &filename)
{
    std::vector<GraphParams> result;

    std::ifstream ifs(filename);
    if (!ifs.is_open())
    {
        result.push_back({100, 500});
        return result;
    }

    std::string line;
    GraphParams current{0, 0};
    bool haveVertices = false;
    bool haveEdges = false;

    while (std::getline(ifs, line))
    {
        if (line.find("NUM_VERTICES=") != std::string::npos)
        {
            current.numVertices = std::stoi(line.substr(line.find('=') + 1));
            haveVertices = true;
        }
        else if (line.find("NUM_EDGES=") != std::string::npos)
        {
            current.numEdges = std::stoi(line.substr(line.find('=') + 1));
            haveEdges = true;
        }

        if (haveVertices && haveEdges)
        {
            result.push_back(current);
            haveVertices = false;
            haveEdges = false;
        }
    }
    ifs.close();

    if (result.empty())
    {
        result.push_back({100, 500});
    }

    return result;
}

static void TestFunctionality_Basic()
{
    std::cout << "== TestFunctionality_Basic ==\n";

    {
        Graph<int> g(true);
        g.AddVertex(1);
        g.AddVertex(2);

        g.AddEdge(1, 2, EdgeProperties(-1, nullptr));

        if (g.GetVerticesCount() != 2)
        {
            std::cerr << "Error: vertices count mismatch!\n";
        }
        if (!g.HasEdge(1, 2))
        {
            std::cerr << "Error: edge (1->2) not found!\n";
        }

        g.RemoveEdge(1, 2);
        if (g.HasEdge(1, 2))
        {
            std::cerr << "Error: edge (1->2) should have been removed!\n";
        }

        g.RemoveVertex(1);
        if (g.GetVerticesCount() != 1)
        {
            std::cerr << "Error: vertex removal failed!\n";
        }
    }

    {
        Graph<int> g(false);
        g.AddVertex(1);
        g.AddVertex(2);

        g.AddEdge(1, 2, EdgeProperties(-1, ShrdPtr<IEdgeInfo>(new MyEdgeInfo(1.0))));
        g.AddEdge(1, 2, EdgeProperties(-1, ShrdPtr<IEdgeInfo>(new MyEdgeInfo(2.0))));

        if (!g.HasEdge(1, 2))
        {
            std::cerr << "Error: (1--2) not found!\n";
        }

        g.UpdateEdge(1, 2, EdgeProperties(-1, ShrdPtr<IEdgeInfo>(new MyEdgeInfo(5.0))));

        double sum = g.GetSumOfEdges(1, 2);

        if (sum != 10.0)
        {
            std::cerr << "Error in UpdateEdge or GetSumOfEdges!\n";
        }
    }

    {
        Graph<int> g(true);
        g.AddVertex(10);
        g.AddVertex(20);

        auto e1 = EdgeProperties(-1, ShrdPtr<IEdgeInfo>(new MyEdgeInfo(3.0)));
        auto e2 = EdgeProperties(-1, ShrdPtr<IEdgeInfo>(new MyEdgeInfo(7.0)));

        g.AddEdge(10, 20, e1);
        g.AddEdge(10, 20, e2);

        g.UpdateEdgeById(10, 20, 0, EdgeProperties(-1, ShrdPtr<IEdgeInfo>(new MyEdgeInfo(9.9))));

        double sum = g.GetSumOfEdges(10, 20);
        if (std::abs(sum - 16.9) > 1e-9)
        {
            std::cerr << "Error in UpdateEdgeById!\n";
        }
    }

    std::cout << "== Basic functional tests passed.\n";
}

static void TestFunctionality_DijkstraSmall()
{
    std::cout << "== TestFunctionality_DijkstraSmall ==\n";

    Graph<int> g(false);

    g.AddVertex(1);
    g.AddVertex(2);
    g.AddVertex(3);

    g.AddEdge(1, 2, EdgeProperties(-1, ShrdPtr<IEdgeInfo>(new MyEdgeInfo(10.0))));
    g.AddEdge(2, 3, EdgeProperties(-1, ShrdPtr<IEdgeInfo>(new MyEdgeInfo(5.0))));
    g.AddEdge(1, 3, EdgeProperties(-1, ShrdPtr<IEdgeInfo>(new MyEdgeInfo(20.0))));

    DijkstraAlgorithm<int> dAlg;
    auto res = dAlg.FindPath(g, 1, 3);

    if (std::abs(res.distance - 15.0) > 1e-9)
    {
        std::cerr << "Error: DijkstraAlgorithm wrong distance (expected 15)!\n";
    }

    if (res.path.GetLength() != 2)
    {
        std::cerr << "Error: DijkstraAlgorithm path should have 2 edges!\n";
    }
    else
    {
        auto &edge0 = res.path.Get(0);
        if (edge0.from != 1 || edge0.to != 2)
        {
            std::cerr << "Error: first edge in path should be 1->2!\n";
        }

        auto &edge1 = res.path.Get(1);
        if (edge1.from != 2 || edge1.to != 3)
        {
            std::cerr << "Error: second edge in path should be 2->3!\n";
        }
    }

    std::cout << "== DijkstraSmall test passed.\n";
}

void TestFunctionality()
{
    std::cout << "=== Functional Tests ===\n";

    TestFunctionality_Basic();

    TestFunctionality_DijkstraSmall();

    std::cout << "Functional tests completed.\n";
}

static void WriteCsvLine(std::ofstream &ofs,
                         int numVertices, int numEdges,
                         double dijkstraMs, double bellmanMs)
{
    ofs << numVertices << ","
        << numEdges << ","
        << dijkstraMs << ","
        << bellmanMs << "\n";
}

void TestPerformance(const std::string &configFile)
{
    auto allParams = LoadAllConfigs(configFile);

    std::ofstream csv("results.csv");
    if (!csv.is_open())
    {
        std::cerr << "Warning: cannot open results.csv for writing.\n";
        return;
    }

    csv << "Vertices,Edges,Dijkstra(ms),BellmanFord(ms)\n";

    std::cout << "=== Performance Tests ===\n";

    srand(12345);

    for (auto &param : allParams)
    {
        int V = param.numVertices;
        int E = param.numEdges;
        std::cout << "Config: vertices=" << V << ", edges=" << E << "\n";

        double sumDijkstraMs = 0.0;
        double sumBellmanMs = 0.0;
        const int experiments = 5;

        for (int exp = 0; exp < experiments; ++exp)
        {
            Graph<int> g(true);

            for (int i = 1; i <= V; ++i)
            {
                g.AddVertex(i);
            }

            for (int i = 0; i < E; ++i)
            {
                int from = 1 + (rand() % V);
                int to = 1 + (rand() % V);
                double w = 1.0 + (rand() % 100);
                g.AddEdge(from, to, EdgeProperties(-1, ShrdPtr<IEdgeInfo>(new MyEdgeInfo(w))));
            }

            {
                DijkstraAlgorithm<int> dAlg;
                auto startTime = std::chrono::steady_clock::now();
                auto res = dAlg.FindPath(g, 1, V);
                auto endTime = std::chrono::steady_clock::now();
                double durationMs =
                    std::chrono::duration<double, std::milli>(endTime - startTime).count();
                sumDijkstraMs += durationMs;
            }

            {
                BellmanFordAlgorithm<int> bfAlg;
                auto startTime = std::chrono::steady_clock::now();
                auto res = bfAlg.FindPath(g, 1, V);
                auto endTime = std::chrono::steady_clock::now();
                double durationMs =
                    std::chrono::duration<double, std::milli>(endTime - startTime).count();
                sumBellmanMs += durationMs;
            }
        }

        double avgDijkstra = sumDijkstraMs / experiments;
        double avgBellman = sumBellmanMs / experiments;

        std::cout << "[Avg Dijkstra] = " << avgDijkstra << " ms, "
                  << "[Avg BellmanFord] = " << avgBellman << " ms\n";

        WriteCsvLine(csv, V, E, avgDijkstra, avgBellman);
    }

    csv.close();
    std::cout << "Performance tests completed. Results in results.csv\n";
}

void runTests()
{
    TestFunctionality();
    TestPerformance("config.txt");
}
