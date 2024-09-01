#include "graph.h"
#include <fstream>

#include "pbrt/util/progressreporter.h"

namespace graph {

char SEP(' ');
char NEW('\n');

// Helper methods
template<class T>
std::ostream& operator<<(std::ostream& out, const Point3<T>& p) {
    out << p.x << SEP << p.y << SEP << p.z << SEP;
    return out;
}

template<class T>
std::istream& operator>>(std::istream& in, Point3<T>& p) {
    in >> p.x >> p.y >> p.z;
    return in;
}

void EdgeData::AddSample(const EdgeData& sample) {
    throughput *= numSamples;
    weightedThroughput *= numSamples;

    throughput += sample.throughput * sample.numSamples;
    weightedThroughput += sample.weightedThroughput * sample.numSamples;

    numSamples += sample.numSamples;

    throughput /= numSamples;
    weightedThroughput /= numSamples;
}

// Graph implementations
template<typename T>
OptRefConst<T> GetByIdConst(int id, const std::unordered_map<int, T>& collection) {
    auto result = collection.find(id);

    if (result == collection.end())
        return {};

    return result->second;
}
OptRefConst<Vertex> Graph::GetVertexConst(int id) const { return GetByIdConst(id, vertices); }
OptRefConst<Edge> Graph::GetEdgeConst(int id) const { return GetByIdConst(id, edges); }
OptRefConst<Path> Graph::GetPathConst(int id) const { return GetByIdConst(id, paths); }

template<typename T>
OptRef<T> GetById(int id,  std::unordered_map<int, T>& collection) {
    auto result = collection.find(id);

    if (result == collection.end())
        return {};

    return result->second;
}
OptRef<Vertex> Graph::GetVertex(int id)  { return GetById(id, vertices); }
OptRef<Edge> Graph::GetEdge(int id)  { return GetById(id, edges); }
OptRef<Path> Graph::GetPath(int id)  { return GetById(id, paths); }

bool Graph::RemoveVertex(int id) {
    auto result = vertices.find(id);
    if (result == vertices.end())
        return false;

    for (auto [_, edgeId] : result->second.inEdges)
        RemoveEdge(edgeId);

    for (auto [_, edgeId] : result->second.outEdges)
        RemoveEdge(edgeId);

    vertices.erase(id);
    return true;
}

OptRef<Edge> Graph::AddEdge(int fromId, int toId, const EdgeData& data) {
    return AddEdge(++curId, fromId, toId, data);
}

OptRef<Edge> Graph::AddEdge(int id, int fromId, int toId, const EdgeData& data) {
    if (edges.find(id) != edges.end())
        ErrorExit("Id already exists");

    auto fromResult = vertices.find(fromId);
    if (fromResult == vertices.end())
        return {};

    auto toResult = vertices.find(toId);
    if (toResult == vertices.end())
        return {};

    Vertex& from = fromResult->second;
    Vertex& to = toResult->second;

    auto edgeResult = to.inEdges.find(from.id);

    if (edgeResult != to.inEdges.end() && from.outEdges.find(to.id) != from.outEdges.end()) {
        OptRef<Edge> edge = GetEdge(edgeResult->second);
        edge.value().get().data.AddSample(data);
        return edge;
    }

    auto emplaceResult = edges.emplace(id, Edge{id, from.id, to.id, data});
    auto& newEdge = emplaceResult.first->second;

    from.outEdges.insert({to.id, newEdge.id});
    to.inEdges.insert({from.id, newEdge.id});

    return newEdge;
}

bool Graph::RemoveEdge(int id) {
    auto result = edges.find(id);
    if (result == edges.end())
        return false;

    auto& [_, from, to, data, edgePaths] = result->second;

    GetVertex(from).value().get().outEdges.erase(to);
    GetVertex(to).value().get().inEdges.erase(from);

    for (auto [pathId, index] : edgePaths)
        paths.erase(pathId); // TODO: consider alternatives

    edges.erase(id);
    return true;
}

Path& Graph::AddPath() {
    return AddPath(++curId);
}

Path& Graph::AddPath(int id) {
    if (paths.find(id) != paths.end())
        ErrorExit("Id already exists");

    auto result = paths.emplace(id, Path{id});
    return result.first->second;
}

bool Graph::RemovePath(int id) {
    auto result = paths.find(id);
    if (result == paths.end())
        return false;

    Path& path = result->second;
    if (!path.edges.empty()) {
        for (auto edgeId : path.edges) {
            Edge& edge = GetEdge(edgeId).value();
            edge.paths.erase(path.id);

            if (edge.paths.empty()) {
                RemoveEdge(edge.id);

                if (Vertex& to = GetVertex(edge.to).value(); to.inEdges.empty() && to.outEdges.empty())
                    RemoveVertex(to.id);
            }
        }

        Vertex& firstVertex = GetVertex(GetEdge(path.edges[0]).value().get().from).value();
        if (firstVertex.inEdges.empty() && firstVertex.outEdges.size() == 1)
            RemoveVertex(firstVertex.id);
    }

    paths.erase(id);
    return true;
}

bool Graph::AddEdgeToPath(int edgeId, int pathId) {
    auto edgeResult = edges.find(edgeId);
    if (edgeResult == edges.end())
        return false;

    auto pathResult = paths.find(pathId);
    if (pathResult == paths.end())
        return false;

    Edge& edge = edgeResult->second;
    Path& path = pathResult->second;

    for (int pathEdgeId : path.edges) {
        if (pathEdgeId == edge.id)
            return false;
    }

    path.edges.push_back(edge.id);

    int index = static_cast<int>(path.edges.size());
    edge.paths.insert({path.id, index});

    return true;
}

inline void WriteEdgeData(std::ostream& out, const EdgeData& data) {
    for (int i = 0; i < NSpectrumSamples; ++i) {
        out << data.throughput[i] << SEP;
    }
    out << data.weightedThroughput << SEP << data.numSamples << SEP;
}

inline EdgeData ReadEdgeData(std::istream& in) {
    std::vector<float> throughputs(NSpectrumSamples);
    for (int i = 0; i < NSpectrumSamples; ++i)
        in >> throughputs[i];

    float weightedThroughput, numSamples;
    in >> weightedThroughput >> numSamples;

    return EdgeData{SampledSpectrum(throughputs), weightedThroughput, numSamples};
}

void Graph::WriteToStream(std::ostream& out, StreamFlags flags) const {
    out << (flags.useCoors ? "True" : "False") << SEP
        << (flags.useThroughput ? "True" : "False") << SEP
        << (flags.useRayVertexTypes ? "True" : "False") << NEW;

    out << curId << SEP <<
           vertices.size() << SEP <<
           edges.size() << SEP <<
           paths.size() << NEW;

    for (auto& [id, vertex] : vertices) {
        out << vertex.id << SEP << vertex.point;

        if (flags.useRayVertexTypes) {
            std::optional<RayVertexType> vertexType = vertex.data.type;
            out << (vertexType.has_value() ? vertexType.value() : -1) << SEP;
        }

        if (flags.useCoors)
            out << vertex.coors.value();
        out << NEW;
    }

    for (auto& [id, edge] : edges) {
        out << edge.id << SEP << edge.from << SEP << edge.to << SEP;
        if (flags.useThroughput)
            WriteEdgeData(out, edge.data);
        out << NEW;
    }

    for (auto& [id, path] : paths) {
        out << path.id << SEP << path.edges.size() << SEP;
        for (auto edgeId : path.edges)
            out << edgeId << SEP;
        out << NEW;
    }
}

void Graph::ReadFromStream(std::istream& in) {
    StreamFlags flags;

    std::string useCoors, useThroughput, useRayVertexType;
    in >> useCoors >> useThroughput >> useRayVertexType;
    if (useCoors == "True")
        flags.useCoors = true;
    if (useThroughput == "True")
        flags.useThroughput = true;
    if (useRayVertexType == "True")
        flags.useRayVertexTypes = true;

    int verticesCap, edgesCap, pathsCap;
    in >> curId >> verticesCap >> edgesCap >> pathsCap;
    vertices.reserve(verticesCap);
    edges.reserve(edgesCap);
    paths.reserve(pathsCap);

    for (int i = 0; i < verticesCap; ++i) {
        int id;
        Point3f point;
        in >> id >> point;

        std::optional<RayVertexType> rayType;

        if (flags.useRayVertexTypes) {
            int type;
            in >> type;

            if (type != -1)
                rayType = static_cast<RayVertexType>(type);
        }

        auto& vertex = AddVertex(id, point, VertexData{rayType});

        if (flags.useCoors) {
            Point3i coors;
            in >> coors;
            vertex.coors = coors;
        }
    }

    for (int i = 0; i < edgesCap; ++i) {
        int id, fromId, toId;
        in >> id >> fromId >> toId;
        EdgeData data = flags.useThroughput ? ReadEdgeData(in) : EdgeData{};

        AddEdge(id, fromId, toId, data);
    }

    for (int i = 0; i < pathsCap; ++i) {
        int id, pathLength;
        in >> id >> pathLength;

        Path& path = AddPath(id);

        for (int j = 0; j < pathLength; ++j) {
            int pathEdgeId;
            in >> pathEdgeId;

            AddEdgeToPath(pathEdgeId, path.id);
        }
    }
}

void Graph::WriteToDisk(const std::string& fileName, const std::string& desc, StreamFlags flags) {
    std::ofstream file(util::FileNameToPath(fileName));
    file << desc << NEW;

    auto asUniform = dynamic_cast<UniformGraph*>(this);
    auto asFree = dynamic_cast<FreeGraph*>(this);
    if (asUniform)
        asUniform->WriteToStream(file, flags);
    if (asFree)
        asFree->WriteToStream(file, flags);

    file.close();
}

void Graph::WriteToDisk(const std::string& fileName, Description desc, StreamFlags flags) {
    WriteToDisk(fileName, GetDescriptionName(desc), flags);
}

util::VerticesHolder Graph::GetVerticesList() const {
    std::vector<std::pair<int, Point3f>> vList;
    vList.reserve(vertices.size());

    std::transform(vertices.begin(), vertices.end(), std::back_inserter(vList),
        [](const std::pair<int, Vertex>& pair) {
            return std::pair{ pair.second.id, pair.second.point };
        });
    return util::VerticesHolder(vList);
}

util::VerticesHolder Graph::GetPathEndsList() const {
    std::vector<std::pair<int, Point3f>> vList;
    vList.reserve(paths.size());

    std::transform(paths.begin(), paths.end(), std::back_inserter(vList),
        [this](const std::pair<int, Path>& pair) {
            const Vertex& pathEnd = GetVertexConst(pair.second.edges.back()).value();
            return std::pair{ pathEnd.id, pathEnd.point };
        });
    return util::VerticesHolder(vList);
}

// UniformGraph implementations
OptRefConst<Vertex> UniformGraph::GetVertexConst(Point3i coors) const {
    auto result = coorsMap.find(coors);
    if (result == coorsMap.end())
        return {};

    return GetVertexConst(result->second);
}

OptRef<Vertex> UniformGraph::GetVertex(Point3i coors) {
    auto result = coorsMap.find(coors);
    if (result == coorsMap.end())
        return {};

    return GetVertex(result->second);
}

Vertex& UniformGraph::AddVertex(Point3f p, const VertexData& data) {
    return AddVertex(++curId, p, data);
}

Vertex& UniformGraph::AddVertex(int id, Point3f p, const VertexData& data) {
    auto [coors, _] = FitToGraph(p);
    return AddVertex(id, coors, data);
}

Vertex& UniformGraph::AddVertex(Point3i coors, const VertexData& data) {
    return AddVertex(++curId, coors, data);
}

Vertex& UniformGraph::AddVertex(int id, Point3i coors, const VertexData& data) {
    if (vertices.find(id) != vertices.end())
        ErrorExit("Id already exists");

    auto findResult = coorsMap.find(coors);
    if (findResult == coorsMap.end()) {
        auto emplaceResult = vertices.emplace(id, Vertex{id, coors*spacing, data, coors});

        Vertex& newVertex = emplaceResult.first->second;
        coorsMap.insert({coors, newVertex.id});

        return newVertex;
    }

    // TODO: merge vertex data
    return GetVertex(findResult->second).value();
}

bool UniformGraph::RemoveVertex(int id) {
    auto result = vertices.find(id);
    if (result == vertices.end())
        return false;

    coorsMap.erase(result->second.coors.value());

    return Graph::RemoveVertex(id);
}

inline std::pair<Point3i, Point3f> UniformGraph::FitToGraph(Point3f p) const {
    Point3i coors;
    for (int i = 0; i < 3; ++i) {
        coors[i] = static_cast<int>(std::round(p[i] / spacing));
    }

    Point3f fittedPoint = coors * spacing;

    return {coors, fittedPoint};
}

void UniformGraph::WriteToStream(std::ostream& out, StreamFlags flags) const {
    out << "uniform" << SEP << spacing << NEW;

    Graph::WriteToStream(out, flags);
}

void UniformGraph::ReadFromStream(std::istream& in) {
    std::string description, name;
    in >> description >> name >> spacing;

    Graph::ReadFromStream(in);
}

UniformGraph UniformGraph::ReadFromDisk(const std::string& fileName) {
    UniformGraph graph;
    std::ifstream file(util::FileNameToPath(fileName));
    graph.ReadFromStream(file);
    return graph;
}

// FreeGraph implementations
Vertex& FreeGraph::AddVertex(Point3f p, const VertexData& data) {
    return AddVertex(++curId, p, data);
}

Vertex& FreeGraph::AddVertex(int id, Point3f p, const VertexData& data) {
    auto emplaceResult = vertices.emplace(id, Vertex{id, p, data});
    return emplaceResult.first->second;
}

UniformGraph FreeGraph::ToUniform(float spacing) const {
    UniformGraph uniform(spacing);

    ProgressReporter progress(static_cast<int>(vertices.size()), "Converting graph to uniform", true);

    for (auto& [id, oldVertex] : vertices) {
        Vertex& curVertex = uniform.AddVertex(oldVertex.point, oldVertex.data);

        for (auto [otherVertexId, edgeId] : oldVertex.inEdges) {
            const Vertex& otherVertex = vertices.at(otherVertexId);
            Vertex& newVertex = uniform.AddVertex(otherVertex.point, otherVertex.data);
            const Edge& oldEdge = GetEdgeConst(edgeId).value();
            uniform.AddEdge(newVertex.id, curVertex.id, oldEdge.data);
        }
        for (auto [otherVertexId, edgeId] : oldVertex.outEdges) {
            const Vertex& otherVertex = vertices.at(otherVertexId);
            Vertex& newVertex = uniform.AddVertex(otherVertex.point, otherVertex.data);
            const Edge& oldEdge = GetEdgeConst(edgeId).value();
            uniform.AddEdge(curVertex.id, newVertex.id, oldEdge.data);
        }

        progress.Update();
    }

    for (auto& [id, oldPath]: paths) {
        Path& newPath = uniform.AddPath();
        for (auto edgeId : oldPath.edges) {
            uniform.AddEdgeToPath(edgeId, newPath.id);
        }
    }

    progress.Done();

    return uniform;
}

void FreeGraph::WriteToStream(std::ostream& out, StreamFlags flags) const {
    out << "free" << NEW;

    Graph::WriteToStream(out, flags);
}

void FreeGraph::ReadFromStream(std::istream& in) {
    std::string description, name;
    in >> description >> name;

    Graph::ReadFromStream(in);
}

FreeGraph FreeGraph::ReadFromDisk(const std::string& fileName) {
    FreeGraph graph;
    std::ifstream file(util::FileNameToPath(fileName));
    graph.ReadFromStream(file);
    return graph;
}

}
