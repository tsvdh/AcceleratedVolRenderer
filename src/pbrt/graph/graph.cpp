#include "graph.h"

#include <filesystem>
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

void VertexData::MergeWithDataFrom(const VertexData& otherData) {
    if (type != none || otherData.type != none
        || lightScalar != -1 || otherData.lightScalar != -1)
        ErrorExit(StringPrintf("Cannot merge type or lightScalar").c_str());

    this->pathContinuePDF.AddSamples(otherData.pathContinuePDF);
    this->pathRemainLength.AddSamples(otherData.pathRemainLength);
}

void Vertex::AddPathIndex(int pathId, int index) {
    if (paths.find(pathId) == paths.end())
        paths[pathId] = {};

    paths[pathId].push_back(index);
}

void Vertex::AddPathIndices(int pathId, const std::vector<int>& indices) {
    if (paths.find(pathId) == paths.end())
        paths[pathId] = {};

    for (int index : indices)
        paths[pathId].push_back(index);
}

void EdgeData::MergeWithDataFrom(const EdgeData& otherData) {
    this->throughput.AddSamples(otherData.throughput);
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

Vertex& Graph::AddVertex(Point3f p, const VertexData& data) {
    return AddVertex(curVertexId, p, data, true);
}

Vertex& Graph::AddVertex(int id, Point3f p, const VertexData& data) {
    return AddVertex(id, p, data, false);
}

bool Graph::RemoveVertex(int id) {
    auto result = vertices.find(id);
    if (result == vertices.end())
        return false;

    std::vector<int> edgesToRemove;
    std::vector<int> pathsToRemove;

    for (auto [_, edgeId] : result->second.inEdges)
        edgesToRemove.push_back(edgeId);

    for (auto [_, edgeId] : result->second.outEdges)
        edgesToRemove.push_back(edgeId);

    for (auto& [pathId, indices] : result->second.paths) {
        pathsToRemove.push_back(pathId);

        Path& path = GetPath(pathId)->get();
        std::vector<int> verticesWithoutVertex = path.vertices;
        for (int index : indices)
            verticesWithoutVertex[index] = -1;

        int newPathId = -1;

        for (int i = 0; i < path.vertices.size(); ++i) {
            if (verticesWithoutVertex[i] != -1) {
                if (newPathId == -1)
                    newPathId = AddPath().id;
                AddVertexToPath(verticesWithoutVertex[i], newPathId);
            }
            else
                newPathId = -1;
        }
    }

    for (int edgeId : edgesToRemove)
        RemoveEdge(edgeId);

    for (int pathId : pathsToRemove)
        RemovePath(pathId);

    vertices.erase(id);
    return true;
}

OptRef<Edge> Graph::AddEdge(int fromId, int toId, const EdgeData& data) {
    return AddEdge(curEdgeId, fromId, toId, data, true);
}

OptRef<Edge> Graph::AddEdge(int id, int fromId, int toId, const EdgeData& data) {
    return AddEdge(id, fromId, toId, data, false);
}

bool Graph::RemoveEdge(int id) {
    auto result = edges.find(id);
    if (result == edges.end())
        return false;

    auto& [_, from, to, data] = result->second;

    GetVertex(from)->get().outEdges.erase(to);
    GetVertex(to)->get().inEdges.erase(from);

    edges.erase(id);
    return true;
}

Path& Graph::AddPath() {
    return AddPath(++curPathId);
}

Path& Graph::AddPath(int id) {
    if (paths.find(id) != paths.end())
        ErrorExit("Path id %i already exists", id);

    auto result = paths.emplace(id, Path{id});
    return result.first->second;
}

bool Graph::RemovePath(int id) {
    auto result = paths.find(id);
    if (result == paths.end())
        return false;

    // TODO: maybe remove contents of path

    for (int vertexId : result->second.vertices) {
        Vertex& vertex = GetVertex(vertexId)->get();
        vertex.paths.erase(id);
    }

    paths.erase(id);
    return true;
}

bool Graph::AddVertexToPath(int vertexId, int pathId) {
    auto vertexResult = vertices.find(vertexId);
    if (vertexResult == vertices.end())
        return false;

    auto pathResult = paths.find(pathId);
    if (pathResult == paths.end())
        return false;

    Vertex& vertex = vertexResult->second;
    Path& path = pathResult->second;

    path.vertices.push_back(vertex.id);

    int index = static_cast<int>(path.vertices.size() - 1);
    vertex.AddPathIndex(path.id, index);

    return true;
}

OptRef<Edge> Graph::AddEdge(int id, int fromId, int toId, const EdgeData& data, bool incrId) {
    if (edges.find(id) != edges.end())
        ErrorExit("Edge id %i already exists", id);

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
        edge->get().data.MergeWithDataFrom(data);
        return edge;
    }

    auto emplaceResult = edges.emplace(id, Edge{id, from.id, to.id, data});
    auto& newEdge = emplaceResult.first->second;

    from.outEdges.emplace(to.id, newEdge.id);
    to.inEdges.emplace(from.id, newEdge.id);

    if (incrId)
        ++curEdgeId;

    return newEdge;
}

inline void WriteVertexData(std::ostream& out, const VertexData& data, StreamFlags flags) {
    if (flags.useRayVertexTypes)
        out << data.type << SEP;

    if (flags.useLighting) {
        out << data.lightScalar << SEP;
    }
}

inline VertexData ReadVertexData(std::istream& in, StreamFlags flags) {
    int rayVertexType = -1;
    if (flags.useRayVertexTypes)
        in >> rayVertexType;

    float lighting = -1;
    if (flags.useLighting) {
        in >> lighting;
    }

    return VertexData{static_cast<RayVertexType>(rayVertexType), lighting};
}

inline void WriteEdgeData(std::ostream& out, EdgeData data, StreamFlags flags) {
    if (flags.useThroughput)
        out << data.throughput.value << SEP << data.throughput.numSamples << SEP;
}

inline EdgeData ReadEdgeData(std::istream& in, StreamFlags flags) {
    float throughput = - 1;
    float numSamples = -1;

    if (flags.useThroughput)
        in >> throughput >> numSamples;

    return EdgeData{throughput, numSamples};
}

void Graph::WriteToStream(std::ostream& out, StreamFlags flags, StreamOptions options) const {
    out << (flags.useCoors ? "True" : "False") << SEP
        << (flags.useThroughput ? "True" : "False") << SEP
        << (flags.useRayVertexTypes ? "True" : "False") << SEP
        << (flags.useLighting ? "True" : "False") << NEW;

    uint64_t numVertices = options.writeVertices ? vertices.size() : 0;
    uint64_t numEdges = options.writeEdges ? edges.size() : 0;
    uint64_t numPaths = options.writePaths ? paths.size() : 0;

    out << curVertexId << SEP
        << curEdgeId << SEP
        << curPathId << SEP
        << numVertices << SEP
        << numEdges << SEP
        << numPaths << NEW;

    int totalWork = static_cast<int>(numVertices + numEdges + numPaths);
    ProgressReporter progress(totalWork, "Writing graph to stream", false);

    if (options.writeVertices) {
        for (auto& [id, vertex] : vertices) {
            out << vertex.id << SEP << vertex.point;

            if (flags.useCoors)
                out << vertex.coors.value();

            WriteVertexData(out, vertex.data, flags);

            out << NEW;

            progress.Update();
        }
    }

    if (options.writeEdges) {
        for (auto& [id, edge] : edges) {
            out << edge.id << SEP << edge.from << SEP << edge.to << SEP;
            WriteEdgeData(out, edge.data, flags);
            out << NEW;

            progress.Update();
        }
    }

    if (options.writePaths) {
        for (auto& [id, path] : paths) {
            out << path.id << SEP << path.vertices.size() << SEP;
            for (int vertexId : path.vertices)
                out << vertexId << SEP;
            out << NEW;

            progress.Update();
        }
    }

    progress.Done();
}

void Graph::ReadFromStream(std::istream& in) {
    StreamFlags flags;

    std::string useCoors, useThroughput, useRayVertexType, useLighting;
    in >> useCoors >> useThroughput >> useRayVertexType >> useLighting;
    if (useCoors == "True")
        flags.useCoors = true;
    if (useThroughput == "True")
        flags.useThroughput = true;
    if (useRayVertexType == "True")
        flags.useRayVertexTypes = true;
    if (useLighting == "True")
        flags.useLighting = true;

    int numVertices, numEdges, numPaths;
    in >> curVertexId >> curEdgeId >> curPathId >> numVertices >> numEdges >> numPaths;
    vertices.reserve(numVertices);
    edges.reserve(numEdges);
    paths.reserve(numPaths);

    ProgressReporter progress(numVertices + numEdges + numPaths, "Reading graph from stream", false);

    for (int i = 0; i < numVertices; ++i) {
        int id;
        Point3f point;
        in >> id >> point;

        VertexData data = ReadVertexData(in, flags);

        auto& vertex = AddVertex(id, point, data);

        if (flags.useCoors) {
            Point3i coors;
            in >> coors;
            vertex.coors = coors;
        }

        progress.Update();
    }

    for (int i = 0; i < numEdges; ++i) {
        int id, fromId, toId;
        in >> id >> fromId >> toId;
        EdgeData data = ReadEdgeData(in, flags);

        AddEdge(id, fromId, toId, data);

        progress.Update();
    }

    for (int i = 0; i < numPaths; ++i) {
        int id, pathLength;
        in >> id >> pathLength;

        Path& path = AddPath(id);

        for (int j = 0; j < pathLength; ++j) {
            int pathVertexId;
            in >> pathVertexId;

            AddVertexToPath(pathVertexId, path.id);
        }

        progress.Update();
    }

    progress.Done();
}

void Graph::WriteToDisk(const std::string& fileName, const std::string& desc, StreamFlags flags, StreamOptions options) {
    std::string fullPath = util::FileNameToPath(fileName);
    std::string parentPath = fullPath.substr(0, fullPath.find_last_of('/'));
    std::filesystem::create_directories(parentPath);

    std::ofstream file(fullPath);
    file << desc << NEW;

    auto asUniform = dynamic_cast<UniformGraph*>(this);
    auto asFree = dynamic_cast<FreeGraph*>(this);
    if (asUniform)
        asUniform->WriteToStream(file, flags, options);
    if (asFree)
        asFree->WriteToStream(file, flags, options);

    file.close();
}

void Graph::WriteToDisk(const std::string& fileName, Description desc, StreamFlags flags, StreamOptions options) {
    WriteToDisk(fileName, GetDescriptionName(desc), flags, options);
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
            const Vertex& pathEnd = GetVertexConst(pair.second.vertices.back()).value();
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

OptRefConst<Vertex> UniformGraph::GetVertexConst(Point3f point) const {
    return GetVertexConst(FitToGraph(point).first);
}

OptRef<Vertex> UniformGraph::GetVertex(Point3f point) {
    return GetVertex(FitToGraph(point).first);
}

Vertex& UniformGraph::AddVertex(Point3i coors, const VertexData& data) {
    return AddVertex(curVertexId, coors, data, true);
}

Vertex& UniformGraph::AddVertex(int id, Point3i coors, const VertexData& data) {
    return AddVertex(id, coors, data, false);
}

bool UniformGraph::RemoveVertex(int id) {
    auto result = vertices.find(id);
    if (result == vertices.end())
        return false;

    coorsMap.erase(result->second.coors.value());

    return Graph::RemoveVertex(id);
}

inline std::pair<Point3i, Point3f> UniformGraph::FitToGraph(Point3f p) const {
    return util::FitToGraph(p, spacing);
}

Vertex& UniformGraph::AddVertex(int id, Point3f p, const VertexData& data, bool incrId) {
    auto [coors, _] = FitToGraph(p);
    return AddVertex(id, coors, data, incrId);
}

Vertex& UniformGraph::AddVertex(int id, Point3i coors, const VertexData& data, bool incrId) {
    if (vertices.find(id) != vertices.end())
        ErrorExit("Vertex id %i already exists", id);

    auto findResult = coorsMap.find(coors);
    if (findResult != coorsMap.end()) {
        // TODO: merge vertex data (how?)
        return GetVertex(findResult->second).value();
    }

    auto emplaceResult = vertices.emplace(id, Vertex{id, coors * spacing, data, coors});

    Vertex& newVertex = emplaceResult.first->second;
    coorsMap.insert({coors, newVertex.id});

    if (incrId)
        ++curVertexId;

    return newVertex;
}

void UniformGraph::WriteToStream(std::ostream& out, StreamFlags flags, StreamOptions options) const {
    out << "uniform" << SEP << spacing << NEW;

    Graph::WriteToStream(out, flags, options);
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
Vertex& FreeGraph::AddVertex(int id, Point3f p, const VertexData& data, bool incrId) {
    auto emplaceResult = vertices.emplace(id, Vertex{id, p, data});

    if (incrId)
        ++curVertexId;

    return emplaceResult.first->second;
}

UniformGraph FreeGraph::ToUniform(float spacing) const {
    UniformGraph uniform(spacing);

    ProgressReporter progress(static_cast<int>(vertices.size()), "Converting graph to uniform", false);

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
        for (auto vertexId : oldPath.vertices) {
            uniform.AddVertexToPath(vertexId, newPath.id);
        }
    }

    progress.Done();

    return uniform;
}

void FreeGraph::WriteToStream(std::ostream& out, StreamFlags flags, StreamOptions options) const {
    out << "free" << NEW;

    Graph::WriteToStream(out, flags, options);
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
