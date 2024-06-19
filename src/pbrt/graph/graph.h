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

struct Vertex;
struct Edge;
struct EdgeData;
struct Path;

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

    std::unordered_map<int, Vertex*> GetVertices() { return vertices; }
    std::unordered_map<int, Edge*> GetEdges() { return edges; }
    std::unordered_map<int, Path*> GetPaths() { return paths; }

    std::optional<Vertex*> GetVertex(int id);
    std::optional<Edge*> GetEdge(int id);

    virtual Vertex* AddVertex(Point3f p) = 0;
    virtual Vertex* AddVertex(int id, Point3f p) = 0;

    std::optional<Edge*> AddEdge(Vertex* from, Vertex* to, EdgeData* data, bool checkValid);
    std::optional<Edge*> AddEdge(int id, int fromId, int toId, EdgeData* data);

    void AddPath(const Path& path);
    Path* AddPath();

    void WriteToDisk(const std::string& fileName, const std::string& desc);

protected:
    void WriteToStream(std::ostream& out, StreamFlags flags);
    void ReadFromStream(std::istream& in);

    std::unordered_map<int, Vertex*> vertices;
    std::unordered_map<int, Edge*> edges;
    std::unordered_map<int, Path*> paths;
    int curId = -1;
};

class UniformGraph : public Graph {
public:
    UniformGraph() : spacing(1.f) {};
    explicit UniformGraph(float spacing) : spacing(spacing) {};
    UniformGraph(UniformGraph& other) = default;

    Vertex* AddVertex(Point3f p) override;
    Vertex* AddVertex(int id, Point3f p) override;

    friend std::ostream& operator<<(std::ostream& out, UniformGraph& v);
    friend std::istream& operator>>(std::istream& in, UniformGraph& v);

private:
    [[nodiscard]] std::tuple<Point3i, Point3f> FitToGraph(const Point3f& p) const;

    float spacing;
    std::unordered_map<std::string, Vertex*> coorsMap;
};

class FreeGraph : public Graph {
public:
    FreeGraph() = default;
    FreeGraph(FreeGraph &other) = default;


    Vertex* AddVertex(Point3f p) override {
        return AddVertex(++curId, p);
    }

    Vertex* AddVertex(int id, pbrt::Point3f p) override {
        auto newVertex = new Vertex{id, p};
        vertices[id] = newVertex;
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
