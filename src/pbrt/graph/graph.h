#pragma once

#include <pbrt/util/vecmath.h>

#include <optional>
#include <vector>

#include "util.h"
#include "deps/nanoflann.hpp"

namespace graph {

using namespace pbrt;

template <typename T>
using Ref = std::reference_wrapper<T>;
template <typename T>
using OptRef = std::optional<Ref<T>>;
template <typename T>
using RefConst = std::reference_wrapper<const T>;
template <typename T>
using OptRefConst = std::optional<RefConst<T>>;

using StaticTreeType = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<Float, util::VerticesHolder>,
    util::VerticesHolder, 3, int>;

using DynamicTreeType = nanoflann::KDTreeSingleIndexDynamicAdaptor<
    nanoflann::L2_Simple_Adaptor<Float, util::VerticesHolder>,
    util::VerticesHolder, 3, int>;

struct Vertex;
struct VertexData;
struct Edge;
struct EdgeData;
struct Path;

enum RayVertexType {
    none,
    absorb,
    scatter,
    null,
    scatter_final
};

struct VertexData {
    RayVertexType type = none;
    float lightScalar = -1;
    int samples = 0;

    void MergeWithDataFrom(const VertexData& otherData);
};

struct Vertex {
    int id = -1;
    Point3f point;
    VertexData data;

    std::optional<Point3i> coors;
    std::map<int, int> inEdges, outEdges; // other vertex ID, edge id
    std::map<int, std::vector<int>> paths; // path ID, indices in path

    bool operator==(const Vertex& other) const {
        return id == other.id;
    }

    void AddPathIndex(int pathId, int index);
    void AddPathIndices(int pathId, const std::vector<int>& indices);
};

struct EdgeData {
    int samples = 0;

    void MergeWithDataFrom(const EdgeData& otherData);
};

struct Edge {
    int id = -1;
    int from;       // vertex id
    int to;         // vertex id
    EdgeData data;

    bool operator==(const Edge& other) const {
        return id == other.id;
    }
};

struct PathData {
    bool forcedEnd = false;
};

struct Path {
    int id = -1;
    PathData data;

    std::vector<int> vertices; // vertex id

    bool operator==(const Path& other) const {
        return id == other.id;
    }

    [[nodiscard]] int Length() const {
        return static_cast<int>(vertices.size());
    }
};

struct StreamFlags {
    bool useCoors = false;
    bool useSamples = false;
    bool useRayVertexTypes = false;
    bool useLighting = false;
};

struct StreamOptions {
    bool writeVertices = true;
    bool writeEdges = true;
    bool writePaths = true;
};

static int curGraphId = -1;

class Graph {
public:
    virtual ~Graph() = default;

    [[nodiscard]] const std::unordered_map<int, Vertex>& GetVerticesConst() const { return vertices; }
    [[nodiscard]] const std::unordered_map<int, Edge>& GetEdgesConst() const { return edges; }
    [[nodiscard]] const std::unordered_map<int, Path>& GetPathsConst() const { return paths; }

    [[nodiscard]] std::unordered_map<int, Vertex>& GetVertices() { return vertices; }
    [[nodiscard]] std::unordered_map<int, Edge>& GetEdges() { return edges; }
    [[nodiscard]] std::unordered_map<int, Path>& GetPaths() { return paths; }

    [[nodiscard]] virtual OptRefConst<Vertex> GetVertexConst(int id) const;
    [[nodiscard]] OptRefConst<Edge> GetEdgeConst(int id) const;
    [[nodiscard]] OptRefConst<Path> GetPathConst(int id) const;

    [[nodiscard]] virtual OptRef<Vertex> GetVertex(int id);
    [[nodiscard]] OptRef<Edge> GetEdge(int id);
    [[nodiscard]] OptRef<Path> GetPath(int id);

    Vertex& AddVertex(Point3f p, const VertexData& data);
    Vertex& AddVertex(int id, Point3f p, const VertexData& data);
    virtual bool RemoveVertex(int id);

    OptRef<Edge> AddEdge(int fromId, int toId, const EdgeData& data);
    OptRef<Edge> AddEdge(int id, int fromId, int toId, const EdgeData& data);
    bool RemoveEdge(int id);

    Path& AddPath(const PathData& data);
    Path& AddPath(int id, const PathData& data);
    bool RemovePath(int id);

    bool AddVertexToPath(int vertexId, int pathId);

    void WriteToDisk(const std::string& fileName, const std::string& desc, StreamFlags flags, StreamOptions options);
    void WriteToDisk(const std::string& fileName, Description desc, StreamFlags flags, StreamOptions options);

    void CheckSequentialIds() const;

    [[nodiscard]] util::VerticesHolder GetVerticesList() const;

    [[nodiscard]] int GetCurVertexId() const { return curVertexId; }
    void SetCurVertexId(int vertexId) { this->curVertexId = vertexId; }
    [[nodiscard]] int GetCurEdgeId() const { return curEdgeId; }
    [[nodiscard]] int GetCurPathId() const { return curPathId; }
    [[nodiscard]] int GetGraphId() const { return graphId; }

protected:
    virtual Vertex& AddVertex(int id, Point3f p, const VertexData& data, bool incrId) = 0;
    OptRef<Edge> AddEdge(int id, int fromId, int toId, const EdgeData& data, bool incrId);

    virtual void WriteToStream(std::ostream& out, StreamFlags flags, StreamOptions options) const;
    virtual void ReadFromStream(std::istream& in);

    std::unordered_map<int, Vertex> vertices; // ID, object
    std::unordered_map<int, Edge> edges;      // ID, object
    std::unordered_map<int, Path> paths;      // ID, object
    int curVertexId = 0;
    int curEdgeId = 0;
    int curPathId = 0;
    int graphId = ++curGraphId;
};

class UniformGraph final : public Graph {
public:
    UniformGraph() = default;
    explicit UniformGraph(float spacing) : spacing(spacing) {}

    [[nodiscard]] float GetSpacing() const { return spacing; }
    [[nodiscard]] std::unordered_map<Point3i, int, util::PointHash> GetCoorsMap() const { return coorsMap; }

    [[nodiscard]] OptRefConst<Vertex> GetVertexConst(int id) const override { return Graph::GetVertexConst(id); }
    [[nodiscard]] OptRefConst<Vertex> GetVertexConst(Point3i coors) const;
    [[nodiscard]] OptRefConst<Vertex> GetVertexConst(Point3f point) const;

    [[nodiscard]] OptRef<Vertex> GetVertex(int id) override { return Graph::GetVertex(id); }
    [[nodiscard]] OptRef<Vertex> GetVertex(Point3i coors);
    [[nodiscard]] OptRef<Vertex> GetVertex(Point3f point);

    using Graph::AddVertex;
    Vertex& AddVertex(Point3i coors, const VertexData& data);
    Vertex& AddVertex(int id, Point3i coors, const VertexData& data);

    bool RemoveVertex(int id) override;

    [[nodiscard]] std::pair<Point3i, Point3f> FitToGraph(Point3f p) const;

    static UniformGraph ReadFromDisk(const std::string& fileName);

    void WriteToStream(std::ostream& out, StreamFlags flags, StreamOptions options) const override;
    void ReadFromStream(std::istream& in) override;

protected:
    Vertex& AddVertex(int id, Point3f p, const VertexData& data, bool incrId) override;
    Vertex& AddVertex(int id, Point3i coors, const VertexData& data, bool incrId);

private:
    float spacing = 1;
    std::unordered_map<Point3i, int, util::PointHash> coorsMap; // coors, vertex id
};

class FreeGraph final : public Graph {
public:
    FreeGraph() = default;
    explicit FreeGraph(float vertexRadius) : vertexRadius(vertexRadius) {}

    using Graph::AddVertex;

    std::optional<float> GetVertexRadius() { return vertexRadius; }

    [[nodiscard]] UniformGraph ToUniform(float spacing) const;

    static FreeGraph ReadFromDisk(const std::string& fileName);

    void WriteToStream(std::ostream& out, StreamFlags flags, StreamOptions options) const override;
    void ReadFromStream(std::istream& in) override;

protected:
    Vertex& AddVertex(int id, Point3f p, const VertexData& data, bool incrId) override;

private:
    std::optional<float> vertexRadius;
};

}
