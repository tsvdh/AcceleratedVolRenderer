#pragma once

#include <vector>
#include <pbrt/util/vecmath.h>
#include <optional>

namespace graph {

struct Edge;

struct Vertex {
    int id = -1;
    pbrt::Point3f point;
    std::optional<pbrt::Point3i> coors;
    std::vector<Edge*> inEdges;
    std::vector<Edge*> outEdges;

    bool operator==(const Vertex& other) const {
        return id == other.id;
    }
};

struct EdgeData {

};

struct Edge {
    int id = -1;
    Vertex* from = nullptr;
    Vertex* to = nullptr;
    EdgeData* data = nullptr;
};

struct Path {
    int id = -1;
    std::vector<Edge*> edges;
};

class Graph {
public:
    Graph() = default;
    Graph(Graph& other) = default;

    std::optional<Vertex*> GetVertex(int id);
    std::optional<Edge*> GetEdge(int id);

    virtual Vertex* AddVertex(pbrt::Point3f& p) = 0;

    std::optional<Edge*> AddEdge(Vertex* from, Vertex* to);
    std::optional<Edge*> AddEdge(Vertex* from, Vertex* to, EdgeData* data);

    void AddPath(Path& path);

protected:
    std::vector<Vertex*> vertices;
    std::vector<Edge*> edges;
    std::vector<Path*> paths;
    int curId = 0;
};

class UniformGraph : public Graph {
public:
    UniformGraph() : spacing(1.f) {};
    explicit UniformGraph(float spacing) : spacing(spacing) {};
    UniformGraph(UniformGraph& other) = default;

    Vertex* AddVertex(pbrt::Point3f& p) override;

private:
    [[nodiscard]] std::tuple<pbrt::Point3i, pbrt::Point3f> FitToGraph(pbrt::Point3f& p) const;

    float spacing;
};

class FreeGraph : public Graph {
public:
    FreeGraph() = default;
    FreeGraph(FreeGraph &other) = default;

    Vertex* AddVertex(pbrt::Point3f& p) override {
        auto newVertex = new Vertex{curId++, p};
        vertices.push_back(newVertex);
        return newVertex;
    }

    [[nodiscard]] UniformGraph* ToUniform(float spacing);
};

class ClusterableGraph : public FreeGraph {
public:
    virtual void Cluster() = 0;
private:
    bool isClustered = false;
};

}
