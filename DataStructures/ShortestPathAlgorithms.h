#ifndef SHORTEST_PATH_ALGORITHMS_H
#define SHORTEST_PATH_ALGORITHMS_H

#include <limits>
#include <unordered_map>
#include <vector>
#include <queue>
#include <stack>
#include <stdexcept>

#include "Graph.h"
template <typename TVertex>
struct PathEdge
{
    TVertex from;
    TVertex to;
    EdgeProperties edgeData;

    PathEdge() = default;
    PathEdge(const TVertex &f, const TVertex &t, const EdgeProperties &e)
        : from(f), to(t), edgeData(e) {}
};

template <typename TVertex>
struct ShortestPathResult
{
    double distance;
    DynamicArraySmart<PathEdge<TVertex>> path;
};

template <typename TVertex>
struct PrevData
{
    TVertex prevVertex;
    EdgeProperties edgeUsed;
};

template <typename TVertex>
class IShortestPathAlgorithm
{
public:
    virtual ShortestPathResult<TVertex> FindPath(const Graph<TVertex> &graph,
                                                 const TVertex &start,
                                                 const TVertex &end) = 0;
    virtual ~IShortestPathAlgorithm() = default;
};

template <typename TVertex>
EdgeProperties GetMinEdgeData(const Graph<TVertex> &graph,
                             const TVertex &u,
                             const TVertex &v)
{
    EdgeProperties bestEdgeData;
    double minW = std::numeric_limits<double>::infinity();

    if (!graph.HasEdge(u, v))
    {
        return bestEdgeData;
    }

    auto &edgesFrom = graph.GetAdjacencyRef().Get(u);
    auto &edgeList = edgesFrom.Get(v);

    for (int i = 0; i < edgeList.GetLength(); ++i)
    {
        const auto &edgeData = edgeList.Get(i);
        if (edgeData.info)
        {
            double w = edgeData.info->GetWeight();
            if (w < minW)
            {
                minW = w;
                bestEdgeData = edgeData;
            }
        }
    }
    return bestEdgeData;
}

template <typename TVertex>
DynamicArraySmart<PathEdge<TVertex>> ReconstructPathEdges(
    const std::unordered_map<TVertex, PrevData<TVertex>> &prev,
    const TVertex &start,
    const TVertex &end)
{
    DynamicArraySmart<PathEdge<TVertex>> result;

    if (prev.find(end) == prev.end() && end != start)
    {
        return result;
    }

    std::stack<PathEdge<TVertex>> stackEdges;
    TVertex current = end;

    while (current != start)
    {
        if (prev.find(current) == prev.end())
        {
            return DynamicArraySmart<PathEdge<TVertex>>();
        }

        auto pd = prev.at(current);
        PathEdge<TVertex> p(pd.prevVertex, current, pd.edgeUsed);
        stackEdges.push(p);
        current = pd.prevVertex;
    }

    while (!stackEdges.empty())
    {
        result.Append(stackEdges.top());
        stackEdges.pop();
    }

    return result;
}

template <typename TVertex>
class DijkstraAlgorithm : public IShortestPathAlgorithm<TVertex>
{
public:
    ShortestPathResult<TVertex> FindPath(const Graph<TVertex> &graph,
                                         const TVertex &start,
                                         const TVertex &end) override
    {
        const double INF = std::numeric_limits<double>::infinity();

        std::unordered_map<TVertex, double> dist;
        std::unordered_map<TVertex, PrevData<TVertex>> prev;

        auto vertices = graph.GetVertices();
        for (int i = 0; i < vertices.GetLength(); ++i)
        {
            dist[vertices.Get(i)] = INF;
        }
        dist[start] = 0.0;

        using DistVertexPair = std::pair<double, TVertex>;
        std::priority_queue<DistVertexPair, std::vector<DistVertexPair>, std::greater<>> pq;
        pq.emplace(dist[start], start);

        while (!pq.empty())
        {
            auto [currentDist, u] = pq.top();
            pq.pop();

            if (u == end)
            {
                break;
            }

            if (currentDist > dist[u])
            {
                continue;
            }

            auto neighbors = graph.GetNeighbors(u);
            for (int i = 0; i < neighbors.GetLength(); ++i)
            {
                TVertex v = neighbors.Get(i);

                EdgeProperties bestEdge = GetMinEdgeData(graph, u, v);
                if (!bestEdge.info)
                {
                    continue;
                }

                double w = bestEdge.info->GetWeight();
                if (dist[u] + w < dist[v])
                {
                    dist[v] = dist[u] + w;
                    prev[v] = {u, bestEdge};
                    pq.emplace(dist[v], v);
                }
            }
        }
        ShortestPathResult<TVertex> result;
        result.distance = (dist.find(end) != dist.end()) ? dist[end] : INF;

        result.path = ReconstructPathEdges(prev, start, end);
        return result;
    }
};

template <typename TVertex>
class BellmanFordAlgorithm : public IShortestPathAlgorithm<TVertex>
{
public:
    ShortestPathResult<TVertex> FindPath(const Graph<TVertex> &graph,
                                         const TVertex &start,
                                         const TVertex &end) override
    {
        const double INF = std::numeric_limits<double>::infinity();

        std::unordered_map<TVertex, double> dist;
        std::unordered_map<TVertex, PrevData<TVertex>> prev;

        auto vertices = graph.GetVertices();
        for (int i = 0; i < vertices.GetLength(); ++i)
        {
            dist[vertices.Get(i)] = INF;
        }
        dist[start] = 0.0;

        int n = vertices.GetLength();

        for (int i = 0; i < n - 1; ++i)
        {
            bool updated = false;

            auto allEdges = graph.GetEdges();
            for (int e = 0; e < allEdges.GetLength(); ++e)
            {
                const auto &edge = allEdges.Get(e);
                TVertex from = edge.from;
                TVertex to = edge.to;
                const auto &edgeList = edge.edges;

                for (int k = 0; k < edgeList.GetLength(); ++k)
                {
                    const auto &edgeData = edgeList.Get(k);
                    if (edgeData.info)
                    {
                        double w = edgeData.info->GetWeight();
                        if (dist[from] < INF && dist[from] + w < dist[to])
                        {
                            dist[to] = dist[from] + w;
                            prev[to] = {from, edgeData};
                            updated = true;
                        }
                    }
                }
            }

            if (!updated)
            {
                break;
            }
        }

        {
            auto allEdges = graph.GetEdges();
            for (int e = 0; e < allEdges.GetLength(); ++e)
            {
                const auto &edge = allEdges.Get(e);
                TVertex from = edge.from;
                TVertex to = edge.to;
                const auto &edgeList = edge.edges;

                for (int k = 0; k < edgeList.GetLength(); ++k)
                {
                    const auto &edgeData = edgeList.Get(k);
                    if (edgeData.info)
                    {
                        double w = edgeData.info->GetWeight();
                        if (dist[from] < INF && dist[from] + w < dist[to])
                        {
                            throw std::runtime_error("Graph contains a negative-weight cycle");
                        }
                    }
                }
            }
        }

        ShortestPathResult<TVertex> result;
        result.distance = (dist.find(end) != dist.end()) ? dist[end] : INF;
        result.path = ReconstructPathEdges(prev, start, end);
        return result;
    }
};

#endif // SHORTEST_PATH_ALGORITHMS_H

