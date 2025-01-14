#ifndef GRAPH_H
#define GRAPH_H

#include "HashTable.h"
#include "DynamicArraySmart.h"
#include "LinkedListSmart.h"
#include "IEdgeInfo.h"
#include "ShrdPtr.h"
#include <stdexcept>
#include <functional>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>

struct EdgeProperties
{
    int id;
    ShrdPtr<IEdgeInfo> info;

    EdgeProperties(int _id = -1, ShrdPtr<IEdgeInfo> ptr = nullptr)
        : id(_id), info(ptr)
    {
    }
};

struct VertexProperties
{
    double x;
    double y;
    VertexProperties(double _x = 0.0, double _y = 0.0)
        : x(_x), y(_y) {}
};

template <typename TVertex>
struct GraphEdge
{
    TVertex from;
    TVertex to;
    DynamicArraySmart<EdgeProperties> edges;

    GraphEdge() : from(), to() {}
    GraphEdge(const TVertex &f, const TVertex &t,
              const DynamicArraySmart<EdgeProperties> &e)
        : from(f), to(t), edges(e) {}
};

template <typename TVertex>
class Graph
{
public:
    Graph(bool directed = false)
        : isDirected(directed), verticesCount(0), edgeIdCounter(0)
    {
    }

    void AddVertex(const TVertex &v, const VertexProperties &vData = VertexProperties())
    {
        if (!adjacency.ContainsKey(v))
        {

            HashTable<TVertex, DynamicArraySmart<EdgeProperties>> emptyAdj;
            adjacency.Add(v, emptyAdj);

            vertexInfo.Add(v, vData);
            ++verticesCount;
        }
        else
        {

            vertexInfo.Update(v, vData);
        }
    }

    void AddEdge(const TVertex &from, const TVertex &to,
                 const EdgeProperties &data)
    {
        if (!adjacency.ContainsKey(from))
        {
            AddVertex(from);
        }
        if (!adjacency.ContainsKey(to))
        {
            AddVertex(to);
        }
        AddEdgeInternal(from, to, data);

        if (!isDirected)
        {
            AddEdgeInternal(to, from, data);
        }
    }

    void RemoveEdge(const TVertex &from, const TVertex &to)
    {
        if (!adjacency.ContainsKey(from))
            throw std::runtime_error("From-vertex not found");

        auto &edgesFrom = adjacency.Get(from);
        if (!edgesFrom.ContainsKey(to))
            throw std::runtime_error("No such edge found");

        edgesFrom.Remove(to);

        if (!isDirected)
        {
            if (!adjacency.ContainsKey(to))
                throw std::runtime_error("To-vertex not found");
            auto &edgesTo = adjacency.Get(to);
            if (edgesTo.ContainsKey(from))
            {
                edgesTo.Remove(from);
            }
        }
    }

    void RemoveEdgeById(const TVertex &from, const TVertex &to, int edgeId)
    {
        if (!adjacency.ContainsKey(from))
            return;

        auto &edgesFrom = adjacency.Get(from);
        if (!edgesFrom.ContainsKey(to))
            return;

        auto &edgeList = edgesFrom.Get(to);
        for (int i = 0; i < edgeList.GetLength(); ++i)
        {
            if (edgeList.Get(i).id == edgeId)
            {
                edgeList.RemoveAt(i);
                break;
            }
        }

        if (!isDirected && adjacency.ContainsKey(to))
        {
            auto &edgesTo = adjacency.Get(to);
            if (edgesTo.ContainsKey(from))
            {
                auto &backEdgeList = edgesTo.Get(from);
                for (int i = 0; i < backEdgeList.GetLength(); ++i)
                {
                    if (backEdgeList.Get(i).id == edgeId)
                    {
                        backEdgeList.RemoveAt(i);
                        break;
                    }
                }
            }
        }
    }

    void UpdateEdge(const TVertex &from, const TVertex &to,
                    const EdgeProperties &data)
    {
        if (!adjacency.ContainsKey(from))
            throw std::runtime_error("From-vertex not found");

        auto &edgesFrom = adjacency.Get(from);
        if (!edgesFrom.ContainsKey(to))
            throw std::runtime_error("Edge not found");

        auto &edgeList = edgesFrom.Get(to);
        for (int i = 0; i < edgeList.GetLength(); ++i)
        {
            edgeList.Get(i) = data;
        }

        if (!isDirected)
        {
            auto &edgesTo = adjacency.Get(to);
            if (edgesTo.ContainsKey(from))
            {
                auto &backEdgeList = edgesTo.Get(from);
                for (int i = 0; i < backEdgeList.GetLength(); ++i)
                {
                    backEdgeList.Get(i) = data;
                }
            }
        }
    }

    void UpdateEdgeById(const TVertex &from, const TVertex &to,
                        int edgeId, const EdgeProperties &data)
    {
        if (!adjacency.ContainsKey(from))
            return;

        auto &edgesFrom = adjacency.Get(from);
        if (!edgesFrom.ContainsKey(to))
            return;

        auto &edgeList = edgesFrom.Get(to);
        for (int i = 0; i < edgeList.GetLength(); ++i)
        {
            if (edgeList.Get(i).id == edgeId)
            {
                edgeList.Get(i) = data;
                break;
            }
        }

        if (!isDirected && adjacency.ContainsKey(to))
        {
            auto &edgesTo = adjacency.Get(to);
            if (edgesTo.ContainsKey(from))
            {
                auto &backEdgeList = edgesTo.Get(from);
                for (int i = 0; i < backEdgeList.GetLength(); ++i)
                {
                    if (backEdgeList.Get(i).id == edgeId)
                    {
                        backEdgeList.Get(i) = data;
                        break;
                    }
                }
            }
        }
    }

    bool HasEdge(const TVertex &from, const TVertex &to) const
    {
        if (!adjacency.ContainsKey(from))
            return false;
        return adjacency.Get(from).ContainsKey(to);
    }

    DynamicArraySmart<TVertex> GetVertices() const
    {
        DynamicArraySmart<TVertex> vertexArray(static_cast<int>(verticesCount));
        auto it = adjacency.GetIterator();
        while (it->MoveNext())
        {
            vertexArray.Append(it->GetCurrentKey());
        }
        return vertexArray;
    }

    DynamicArraySmart<TVertex> GetNeighbors(const TVertex &v) const
    {
        DynamicArraySmart<TVertex> neighbors;
        if (!adjacency.ContainsKey(v))
        {
            return neighbors;
        }
        const auto &edgesFrom = adjacency.Get(v);
        auto it = edgesFrom.GetIterator();
        while (it->MoveNext())
        {
            neighbors.Append(it->GetCurrentKey());
        }
        return neighbors;
    }

    DynamicArraySmart<GraphEdge<TVertex>> GetEdges() const
    {
        DynamicArraySmart<GraphEdge<TVertex>> edgeList;
        auto vertIt = adjacency.GetIterator();
        while (vertIt->MoveNext())
        {
            auto from = vertIt->GetCurrentKey();
            const auto &edgesFrom = vertIt->GetCurrentValue();

            auto edgeIt = edgesFrom.GetIterator();
            while (edgeIt->MoveNext())
            {
                auto to = edgeIt->GetCurrentKey();
                const auto &edList = edgeIt->GetCurrentValue();
                GraphEdge<TVertex> gEdge(from, to, edList);
                edgeList.Append(gEdge);
            }
        }
        return edgeList;
    }

    VertexProperties GetVertexProperties(const TVertex &v) const
    {
        if (!vertexInfo.ContainsKey(v))
            throw std::runtime_error("Vertex not found");
        return vertexInfo.Get(v);
    }
    void UpdateVertexProperties(const TVertex &v, const VertexProperties &vData)
    {
        if (!vertexInfo.ContainsKey(v))
            throw std::runtime_error("Vertex not found");
        vertexInfo.Update(v, vData);
    }

    void RemoveVertex(const TVertex &v)
    {
        if (!adjacency.ContainsKey(v))
            throw std::runtime_error("Vertex not found");

        auto vertIt = adjacency.GetIterator();
        while (vertIt->MoveNext())
        {
            auto curVert = vertIt->GetCurrentKey();
            if (curVert != v)
            {
                auto &edgesFrom = adjacency.Get(curVert);
                if (edgesFrom.ContainsKey(v))
                {
                    edgesFrom.Remove(v);
                }
            }
        }

        adjacency.Remove(v);
        vertexInfo.Remove(v);
        --verticesCount;
    }

    bool IsDirected() const
    {
        return isDirected;
    }

    size_t GetVerticesCount() const
    {
        return verticesCount;
    }

    double GetSumOfEdges(const TVertex &from, const TVertex &to) const
    {
        if (!HasEdge(from, to))
            throw std::runtime_error("No edges from->to");

        const auto &edgesFrom = adjacency.Get(from);
        const auto &edgeList = edgesFrom.Get(to);

        double sum = 0.0;
        for (int i = 0; i < edgeList.GetLength(); ++i)
        {
            auto &edgeData = edgeList.Get(i);
            if (edgeData.info)
            {
                sum += edgeData.info->GetWeight();
            }
        }
        return sum;
    }

    const HashTable<TVertex, HashTable<TVertex, DynamicArraySmart<EdgeProperties>>> &
    GetAdjacencyRef() const
    {
        return adjacency;
    }

    void SaveToJson(const std::string &filename) const
    {
        nlohmann::json j;
        j["directed"] = isDirected;

        {
            auto arr = nlohmann::json::array();
            auto vertIt = adjacency.GetIterator();
            while (vertIt->MoveNext())
            {
                TVertex v = vertIt->GetCurrentKey();
                const auto &vdata = vertexInfo.Get(v);
                nlohmann::json vObj;
                vObj["id"] = v;
                vObj["x"] = vdata.x;
                vObj["y"] = vdata.y;
                arr.push_back(vObj);
            }
            j["vertices"] = arr;
        }

        {
            auto edgesArr = nlohmann::json::array();
            auto edgeList = GetEdges();
            for (int i = 0; i < edgeList.GetLength(); ++i)
            {
                const auto &gEdge = edgeList.Get(i);
                auto &arrOfDatas = gEdge.edges;

                for (int k = 0; k < arrOfDatas.GetLength(); k++)
                {
                    const auto &ed = arrOfDatas.Get(k);
                    nlohmann::json eObj;
                    eObj["from"] = gEdge.from;
                    eObj["to"] = gEdge.to;
                    eObj["id"] = ed.id;
                    eObj["weight"] = (ed.info ? ed.info->GetWeight() : 0.0);
                    edgesArr.push_back(eObj);
                }
            }
            j["edges"] = edgesArr;
        }

        std::ofstream ofs(filename);
        if (!ofs.is_open())
        {
            throw std::runtime_error("Failed to open file for writing: " + filename);
        }
        ofs << j.dump(4);
        ofs.close();
    }

    void LoadFromJson(const std::string &filename)
    {

        adjacency = HashTable<TVertex, HashTable<TVertex, DynamicArraySmart<EdgeProperties>>>();
        vertexInfo = HashTable<TVertex, VertexProperties>();

        verticesCount = 0;
        edgeIdCounter = 0;

        std::ifstream ifs(filename);
        if (!ifs.is_open())
        {
            throw std::runtime_error("Failed to open file for reading: " + filename);
        }
        nlohmann::json j;
        ifs >> j;
        ifs.close();

        isDirected = j.value("directed", false);

        if (j.contains("vertices"))
        {
            for (auto &vObj : j["vertices"])
            {

                TVertex v = vObj["id"].get<TVertex>();
                double x = vObj.value("x", 0.0);
                double y = vObj.value("y", 0.0);
                AddVertex(v, VertexProperties(x, y));
            }
        }

        if (j.contains("edges"))
        {
            for (auto &eObj : j["edges"])
            {
                TVertex from = eObj["from"].get<TVertex>();
                TVertex to = eObj["to"].get<TVertex>();
                int id = eObj.value("id", -1);
                double w = eObj.value("weight", 0.0);

                EdgeProperties edgeData(-1, nullptr);
                edgeData.id = id;

                edgeData.info = nullptr;

                AddEdge(from, to, edgeData);
            }
        }
    }

    void ExportToDot(const std::string &filename) const
    {
        std::ofstream ofs(filename);
        if (!ofs.is_open())
        {
            throw std::runtime_error("Failed to open file for writing dot: " + filename);
        }

        if (isDirected)
        {
            ofs << "digraph G {\n";

            ofs << "    rankdir=LR;\n";
        }
        else
        {
            ofs << "graph G {\n";

            ofs << "    layout=neato;\n";
        }

        ofs << "    bgcolor=\"white\";\n"
            << "    node [shape=circle, style=filled, fillcolor=\"lightblue\", fontcolor=\"black\"];\n"
            << "    edge [color=\"gray\", fontcolor=\"red\", penwidth=1.5];\n";

        auto vertices = GetVertices();
        for (int i = 0; i < vertices.GetLength(); i++)
        {
            auto v = vertices.Get(i);
            auto vd = vertexInfo.Get(v);

            ofs << "    \"" << v << "\" "
                << "[label=\"" << v << "\", pos=\"" << vd.x << "," << vd.y << "!\"];\n";
        }

        auto allEdges = GetEdges();
        for (int i = 0; i < allEdges.GetLength(); i++)
        {
            const auto &gEdge = allEdges.Get(i);
            auto from = gEdge.from;
            auto to = gEdge.to;

            for (int k = 0; k < gEdge.edges.GetLength(); k++)
            {
                const auto &ed = gEdge.edges.Get(k);

                double w = 0.0;
                if (ed.info)
                {
                    w = ed.info->GetWeight();
                }

                std::string labelText = "id=" + std::to_string(ed.id) + ", w=" + std::to_string(w);

                if (isDirected)
                {
                    ofs << "    \"" << from << "\" -> \"" << to << "\" "
                        << "[label=\"" << labelText << "\", "
                        << "color=\"blue\", "
                        << "fontcolor=\"red\", "
                        << "penwidth=2.0];\n";
                }
                else
                {
                    ofs << "    \"" << from << "\" -- \"" << to << "\" "
                        << "[label=\"" << labelText << "\", "
                        << "color=\"blue\", "
                        << "fontcolor=\"red\", "
                        << "penwidth=2.0];\n";
                }
            }
        }

        ofs << "}\n";
        ofs.close();
    }

private:
    bool isDirected;
    size_t verticesCount;
    int edgeIdCounter;

    HashTable<TVertex, HashTable<TVertex, DynamicArraySmart<EdgeProperties>>> adjacency;

    HashTable<TVertex, VertexProperties> vertexInfo;

    void AddEdgeInternal(const TVertex &from, const TVertex &to,
                         const EdgeProperties &data)
    {
        auto &edgesFrom = adjacency.Get(from);

        if (!edgesFrom.ContainsKey(to))
        {
            edgesFrom.Add(to, DynamicArraySmart<EdgeProperties>());
        }
        auto &edgeList = edgesFrom.Get(to);

        EdgeProperties newData = data;
        newData.id = edgeIdCounter++;
        edgeList.Append(newData);
    }
};

#endif
