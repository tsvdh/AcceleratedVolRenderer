#pragma once

#include <pbrt/util/spectrum.h>
#include <pbrt/util/vecmath.h>

#include <optional>
#include <vector>

#include "util.h"

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

struct Vertex;
struct VertexData;
struct Edge;
struct EdgeData;
struct Path;

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

struct Vertex {
    int id = -1;
    Point3f point;
    VertexData data;
    std::optional<Point3i> coors;
    std::unordered_map<int, int> inEdges, outEdges; // other vertex ID, edge id

    bool operator==(const Vertex& other) const {
        return id == other.id;
    }
};

struct EdgeData {
    SampledSpectrum throughput;    // average of samples
    float weightedThroughput = -1; // average of samples
    float numSamples = 1;

    void AddSample(const EdgeData& sample);
};

struct Edge {
    int id = -1;
    int from;       // vertex id
    int to;         // vertex id
    EdgeData data;
    std::unordered_map<int, int> paths; // path ID, index in path

    bool operator==(const Edge& other) const {
        return id == other.id;
    }
};

struct Path {
    int id = -1;
    std::vector<int> edges; // edge id

    bool operator==(const Path& other) const {
        return id == other.id;
    }
};

struct StreamFlags {
    bool useCoors = false;
    bool useThroughput = false;
    bool useRayVertexTypes = false;
};

class Graph {
public:
    virtual ~Graph() = default;

    [[nodiscard]] const std::unordered_map<int, Vertex>& GetVertices() const { return vertices; }
    [[nodiscard]] const std::unordered_map<int, Edge>& GetEdges() const { return edges; }
    [[nodiscard]] const std::unordered_map<int, Path>& GetPaths() const { return paths; }

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

    Path& AddPath();
    Path& AddPath(int id);
    bool RemovePath(int id);

    bool AddEdgeToPath(int edgeId, int pathId);

    void WriteToDisk(const std::string& fileName, const std::string& desc, StreamFlags flags);
    void WriteToDisk(const std::string& fileName, Description desc, StreamFlags flags);

    [[nodiscard]] util::VerticesHolder GetVerticesList() const;
    [[nodiscard]] util::VerticesHolder GetPathEndsList() const;

protected:
    virtual Vertex& AddVertex(int id, Point3f p, const VertexData& data, bool incrId) = 0;
    OptRef<Edge> AddEdge(int id, int fromId, int toId, const EdgeData& data, bool incrId);

    virtual void WriteToStream(std::ostream& out, StreamFlags flags) const;
    virtual void ReadFromStream(std::istream& in);

    std::unordered_map<int, Vertex> vertices; // ID, object
    std::unordered_map<int, Edge> edges;      // ID, object
    std::unordered_map<int, Path> paths;      // ID, object
    int curId = 0;
};

class UniformGraph final : public Graph {
public:
    UniformGraph() = default;
    explicit UniformGraph(float spacing) : spacing(spacing) {};

    [[nodiscard]] float GetSpacing() const { return spacing; }
    [[nodiscard]] std::unordered_map<Point3i, int, util::PointHash> GetCoorsMap() const { return coorsMap; }

    [[nodiscard]] OptRefConst<Vertex> GetVertexConst(int id) const override { return Graph::GetVertexConst(id); }
    [[nodiscard]] OptRefConst<Vertex> GetVertexConst(Point3i coors) const;

    [[nodiscard]] OptRef<Vertex> GetVertex(int id) override { return Graph::GetVertex(id); }
    [[nodiscard]] OptRef<Vertex> GetVertex(Point3i coors);

    using Graph::AddVertex;
    Vertex& AddVertex(Point3i coors, const VertexData& data);
    Vertex& AddVertex(int id, Point3i coors, const VertexData& data);

    bool RemoveVertex(int id) override;

    [[nodiscard]] std::pair<Point3i, Point3f> FitToGraph(Point3f p) const;

    static UniformGraph ReadFromDisk(const std::string& fileName);

    void WriteToStream(std::ostream& out, StreamFlags flags) const override;
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
    using Graph::AddVertex;

    [[nodiscard]] UniformGraph ToUniform(float spacing) const;

    static FreeGraph ReadFromDisk(const std::string& fileName);

    void WriteToStream(std::ostream& out, StreamFlags flags) const override;
    void ReadFromStream(std::istream& in) override;

protected:
    Vertex& AddVertex(int id, Point3f p, const VertexData& data, bool incrId) override;
};

}
