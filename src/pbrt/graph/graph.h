#pragma once

#include <vector>
#include <optional>
#include <iostream>
#include <fstream>

#include <pbrt/util/vecmath.h>
#include <pbrt/util/spectrum.h>
#include <pbrt/lights.h>

namespace graph {

using namespace pbrt;

struct Edge;

struct Vertex {
    int id = -1;
    Point3f point;
    std::optional<Point3i> coors;
    std::vector<Edge*> inEdges, outEdges;

    // explicit Vertex(Point3f point) : point(point) {}

    bool operator==(const Vertex& other) const {
        return id == other.id;
    }
};

struct EdgeData {
    SampledSpectrum L{0.f}, beta{1.f}, r_u{1.f}, r_l{1.f};
    bool specularBounce = false, anyNonSpecularBounces = false;
    int depth = 0;
    Float etaScale = 1;
    LightSampleContext prevIntrContext;
    
    EdgeData() = default;
    EdgeData(EdgeData& edgeData) = default;
};

struct Edge {
    int id = -1;
    Vertex* from = nullptr;
    Vertex* to = nullptr;
    EdgeData* data = nullptr;

    // explicit Edge(Vertex* from, Vertex* to, EdgeData* data)
    //     : from(from), to(to), data(data) {}

    bool operator==(const Edge& other) const {
        return id == other.id;
    }
};

struct Path {
    int id = -1;
    std::vector<Edge*> edges;

    bool operator==(const Path& other) const {
        return id == other.id;
    }
};

struct StreamFlags {
    bool useCoors = false;
};

class Graph {
public:
    Graph() = default;
    Graph(Graph& other) = default;

    std::optional<Vertex*> GetVertex(int id);
    std::optional<Edge*> GetEdge(int id);

    virtual Vertex* AddVertex(Point3f& p) = 0;

    std::optional<Edge*> AddEdge(Vertex* from, Vertex* to, EdgeData* data, bool checkValid);
    std::optional<Edge*> AddEdge(int id, int fromId, int toId, EdgeData* data);

    void AddPath(Path& path);
    Path* AddPath();

protected:
    void WriteToStream(std::ostream& out, StreamFlags flags);
    void ReadFromStream(std::istream& in);

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

    Vertex* AddVertex(Point3f& p) override;

    friend std::ostream& operator<<(std::ostream& out, UniformGraph& v);
    friend std::istream& operator>>(std::istream& in, UniformGraph& v);

private:
    [[nodiscard]] std::tuple<Point3i, Point3f> FitToGraph(Point3f& p) const;

    float spacing;
};

class FreeGraph : public Graph {
public:
    FreeGraph() = default;
    FreeGraph(FreeGraph &other) = default;

    Vertex* AddVertex(Point3f& p) override {
        auto newVertex = new Vertex{curId++, p};
        vertices.push_back(newVertex);
        return newVertex;
    }

    [[nodiscard]] UniformGraph* ToUniform(float spacing);

    friend std::ostream& operator<<(std::ostream& out, FreeGraph& v);
    friend std::istream& operator>>(std::istream& in, FreeGraph& v);
};

class ClusterableGraph : public FreeGraph {
public:
    virtual void Cluster() = 0;
private:
    bool isClustered = false;
};

}
