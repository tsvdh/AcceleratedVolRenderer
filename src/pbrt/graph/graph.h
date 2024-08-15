#pragma once

#include <pbrt/util/spectrum.h>
#include <pbrt/util/vecmath.h>

#include <optional>
#include <unordered_set>
#include <vector>

#include "util.h"

namespace graph {

using namespace pbrt;

struct Vertex;
struct VertexData;
struct Edge;
struct EdgeData;
struct Path;

struct Vertex {
    int id = -1;
    Point3f point;
    VertexData* data;
    std::optional<Point3i> coors;
    std::unordered_set<Edge*> inEdges, outEdges;

    bool operator==(const Vertex& other) const {
        return id == other.id;
    }
};

enum RayVertexType {
    absorp,
    scatter,
    null,
    entry,
    scatter_final
};

struct VertexData {
    std::optional<RayVertexType> type;
};

struct Edge {
    int id = -1;
    Vertex* from = nullptr;
    Vertex* to = nullptr;
    EdgeData* data = nullptr;
    std::unordered_map<Path*, int> paths; // path, index in path

    bool operator==(const Edge& other) const {
        return id == other.id;
    }
};

struct EdgeData {
    SampledSpectrum throughput;
    float weightedThroughput = -1;
};

struct Path {
    int id = -1;
    std::vector<Edge*> edges;
    std::vector<EdgeData*> edgeData;

    bool operator==(const Path& other) const {
        return id == other.id;
    }
};

struct StreamFlags {
    bool useCoors = false;
    bool useThroughput = false;
    bool useRayVertexTypes = false;
};

inline std::string FileNameToPath(const std::string& fileName) {
    return "files/graphs/" + fileName + ".txt";
}

enum Description {
    basic,
    surface,
    paths,
    search_queue,
    search_surface
};
const std::vector<std::string> descriptionNames{
    "basic",
    "surface",
    "paths",
    "search_queue",
    "search_surface"
};

inline std::string GetDescriptionName(Description desc) {
    return descriptionNames[desc];
}

class Graph {
public:
    Graph() = default;
    Graph(Graph& other) = default;
    virtual ~Graph() = default;

    std::unordered_map<int, Vertex*> GetVertices() { return vertices; }
    std::unordered_map<int, Edge*> GetEdges() { return edges; }
    std::unordered_map<int, Path*> GetPaths() { return paths; }

    virtual std::optional<Vertex*> GetVertex(int id);
    std::optional<Edge*> GetEdge(int id);
    std::optional<Path*> GetPath(int id);

    virtual Vertex* AddVertex(Point3f p, VertexData* data) = 0;
    virtual Vertex* AddVertex(int id, Point3f p, VertexData* data) = 0;
    virtual bool RemoveVertex(int id);

    std::optional<Edge*> AddEdge(Vertex* from, Vertex* to, EdgeData* data, bool checkValid);
    std::optional<Edge*> AddEdge(int id, int fromId, int toId, EdgeData* data);
    bool RemoveEdge(int id);

    void AddPath(const Path& path);
    Path* AddPath();
    bool RemovePath(int id);

    static bool AddEdgeToPath(Edge* edge, EdgeData* data, Path* path);

    void WriteToDisk(const std::string& fileName, const std::string& desc, StreamFlags flags);
    void WriteToDisk(const std::string& fileName, Description desc, StreamFlags flags);

    [[nodiscard]] size_t kdtree_get_point_count() const { return vertices.size(); }
    [[nodiscard]] Float kdtree_get_pt(size_t index, size_t dim) const;
    // ReSharper disable once CppMemberFunctionMayBeStatic
    template <class BBOX> bool kdtree_get_bbox(BBOX& bb) const { return false; }

protected:
    virtual void WriteToStream(std::ostream& out, StreamFlags flags);
    virtual void ReadFromStream(std::istream& in);

    std::unordered_map<int, Vertex*> vertices; // ID, object
    std::unordered_map<int, Edge*> edges;      // ID, object
    std::unordered_map<int, Path*> paths;      // ID, object
    int curId = -1;
};

class UniformGraph final : public Graph {
public:
    UniformGraph() : spacing(1.f) {};
    explicit UniformGraph(float spacing) : spacing(spacing) {};

    float GetSpacing() { return spacing; } // NOLINT(*-make-member-function-const)
    std::unordered_map<Point3i, Vertex*, util::PointHash> GetCoorsMap() { return coorsMap; }

    std::optional<Vertex*> GetVertex(int id) override { return Graph::GetVertex(id); }
    std::optional<Vertex*> GetVertex(Point3i coors);

    Vertex* AddVertex(Point3f p, VertexData*) override;
    Vertex* AddVertex(int id, Point3f p, VertexData* data) override;
    Vertex* AddVertex(Point3i coors, VertexData* data);

    bool RemoveVertex(int id) override;

    [[nodiscard]] std::tuple<Point3i, Point3f> FitToGraph(const Point3f& p) const;

    static UniformGraph* ReadFromDisk(const std::string& fileName);

    void WriteToStream(std::ostream& out, StreamFlags flags) override;
    void ReadFromStream(std::istream& in) override;

private:
    float spacing;
    std::unordered_map<Point3i, Vertex*, util::PointHash> coorsMap; // coors, vertex
};

class FreeGraph : public Graph {
public:
    FreeGraph() = default;

    Vertex* AddVertex(Point3f p, VertexData* data) override {
        return AddVertex(++curId, p, data);
    }

    Vertex* AddVertex(int id, Point3f p, VertexData* data) override {
        auto newVertex = new Vertex{id, p, data};
        vertices[id] = newVertex;
        return newVertex;
    }

    [[nodiscard]] UniformGraph* ToUniform(float spacing);

    static FreeGraph* ReadFromDisk(const std::string& fileName);

    void WriteToStream(std::ostream& out, StreamFlags flags) override;
    void ReadFromStream(std::istream& in) override;
};

class ClusterableGraph : public FreeGraph {
public:
    virtual void Cluster() = 0;
private:
    bool isClustered = false;
};

}
