#include "graph.h"

#include <filesystem>
#include <fstream>
#include <set>

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
        ErrorExit(StringPrintf("Cannot merge vertex type or lightScalar").c_str());

    this->samples += otherData.samples;
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
    this->samples += otherData.samples;
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

    std::set<int> edgesToRemove;
    std::vector<int> pathsToRemove;

    for (auto [_, edgeId] : result->second.inEdges)
        edgesToRemove.insert(edgeId);

    for (auto [_, edgeId] : result->second.outEdges)
        edgesToRemove.insert(edgeId);

    for (auto& [pathId, indices] : result->second.paths) {
        pathsToRemove.push_back(pathId);

        Path& path = GetPath(pathId)->get();
        std::vector<int> verticesWithoutPath = path.vertices;
        for (int index : indices)
            verticesWithoutPath[index] = -1;

        int newPathId = -1;

        for (int i = 0; i < path.vertices.size(); ++i) {
            if (verticesWithoutPath[i] != -1) {
                if (newPathId == -1)
                    newPathId = AddPath(path.data).id;
                AddVertexToPath(verticesWithoutPath[i], newPathId);
            }
            else {
                GetPath(newPathId)->get().data.forcedEnd = true;
                newPathId = -1;
            }
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

Path& Graph::AddPath(const PathData& data) {
    return AddPath(++curPathId, data);
}

Path& Graph::AddPath(int id, const PathData& data) {
    if (paths.find(id) != paths.end())
        ErrorExit("Path id %i already exists", id);

    auto result = paths.emplace(id, Path{id, data});
    return result.first->second;
}

bool Graph::RemovePath(int id) {
    auto result = paths.find(id);
    if (result == paths.end())
        return false;

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
    Edge& newEdge = emplaceResult.first->second;

    from.outEdges.emplace(to.id, newEdge.id);
    to.inEdges.emplace(from.id, newEdge.id);

    if (incrId)
        ++curEdgeId;

    return newEdge;
}

inline void WriteVertexData(std::ostream& out, const VertexData& data, StreamFlags flags) {
    if (flags.useRayVertexTypes)
        out << data.type << SEP;

    if (flags.useLighting)
        out << data.lightScalar << SEP;

    if (flags.useSamples)
        out << data.samples << SEP;

    if (flags.useRenderSearchRange)
        out << data.renderSearchRange << SEP;
}

inline VertexData ReadVertexData(std::istream& in, StreamFlags flags) {
    int rayVertexType = -1;
    if (flags.useRayVertexTypes)
        in >> rayVertexType;

    float lightScalar = -1;
    if (flags.useLighting)
        in >> lightScalar;

    int samples = -1;
    if (flags.useSamples)
        in >> samples;

    float renderSearchRange = -1;
    if (flags.useRenderSearchRange)
        in >> renderSearchRange;

    return VertexData{static_cast<RayVertexType>(rayVertexType), lightScalar, samples, renderSearchRange};
}

inline void WriteEdgeData(std::ostream& out, EdgeData data, StreamFlags flags) {
    if (flags.useSamples)
        out << data.samples << SEP;
}

inline EdgeData ReadEdgeData(std::istream& in, StreamFlags flags) {
    int samples = -1;
    if (flags.useSamples)
        in >> samples;

    return EdgeData{samples};
}

void Graph::WriteToStream(std::ostream& out) const {
    if (!streamOptions || !streamFlags)
        ErrorExit("Graph stream options or flags not set");

    out << (streamFlags->useCoors ? "True" : "False") << SEP
        << (streamFlags->useSamples ? "True" : "False") << SEP
        << (streamFlags->useRayVertexTypes ? "True" : "False") << SEP
        << (streamFlags->useLighting ? "True" : "False") << SEP
        << (streamFlags->useRenderSearchRange ? "True" : "False") << NEW;

    uint64_t numVertices = streamOptions->writeVertices ? vertices.size() : 0;
    uint64_t numEdges = streamOptions->writeEdges ? edges.size() : 0;
    uint64_t numPaths = streamOptions->writePaths ? paths.size() : 0;

    out << curVertexId << SEP
        << curEdgeId << SEP
        << curPathId << SEP
        << numVertices << SEP
        << numEdges << SEP
        << numPaths << NEW;

    int totalWork = static_cast<int>(numVertices + numEdges + numPaths);
    ProgressReporter progress(totalWork, "Writing graph to stream", false);

    if (streamOptions->writeVertices) {
        for (auto& [id, vertex] : vertices) {
            out << vertex.id << SEP << vertex.point;

            WriteVertexData(out, vertex.data, streamFlags.value());

            if (streamFlags->useCoors)
                out << vertex.coors.value();

            out << NEW;

            progress.Update();
        }
    }

    if (streamOptions->writeEdges) {
        for (auto& [id, edge] : edges) {
            out << edge.id << SEP << edge.from << SEP << edge.to << SEP;
            WriteEdgeData(out, edge.data, streamFlags.value());
            out << NEW;

            progress.Update();
        }
    }

    if (streamOptions->writePaths) {
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
    std::string useCoors, useSamples, useRayVertexType, useLighting, useSearchRange;
    in >> useCoors >> useSamples >> useRayVertexType >> useLighting >> useSearchRange;

    streamFlags = StreamFlags{};
    if (useCoors == "True")
        streamFlags->useCoors = true;
    if (useSamples == "True")
        streamFlags->useSamples = true;
    if (useRayVertexType == "True")
        streamFlags->useRayVertexTypes = true;
    if (useLighting == "True")
        streamFlags->useLighting = true;
    if (useSearchRange == "True")
        streamFlags->useRenderSearchRange = true;

    int numVertices, numEdges, numPaths;
    in >> curVertexId >> curEdgeId >> curPathId >> numVertices >> numEdges >> numPaths;
    vertices.reserve(numVertices);
    edges.reserve(numEdges);
    paths.reserve(numPaths);

    streamOptions = StreamOptions{numVertices > 0, numEdges > 0, numPaths > 0};

    ProgressReporter progress(numVertices + numEdges + numPaths, "Reading graph from stream", false);
    for (int i = 0; i < numVertices; ++i) {
        int id;
        Point3f point;
        in >> id >> point;

        VertexData data = ReadVertexData(in, streamFlags.value());

        auto& vertex = AddVertex(id, point, data);

        if (streamFlags->useCoors) {
            Point3i coors;
            in >> coors;
            vertex.coors = coors;
        }

        progress.Update();
    }

    for (int i = 0; i < numEdges; ++i) {
        int id, fromId, toId;
        in >> id >> fromId >> toId;
        EdgeData data = ReadEdgeData(in, streamFlags.value());

        AddEdge(id, fromId, toId, data);

        progress.Update();
    }

    for (int i = 0; i < numPaths; ++i) {
        int id, pathLength;
        in >> id >> pathLength;

        Path& path = AddPath(id, PathData{});

        for (int j = 0; j < pathLength; ++j) {
            int pathVertexId;
            in >> pathVertexId;

            AddVertexToPath(pathVertexId, path.id);
        }

        progress.Update();
    }
    progress.Done();
}

void Graph::WriteToDisk(const std::string& fileName, const std::string& desc) {
    std::string fullPath = util::FileNameToPath(fileName);
    util::CreateParentDirectories(fullPath);

    std::ofstream file(fullPath);
    file << desc << NEW;

    auto asUniform = dynamic_cast<UniformGraph*>(this);
    auto asFree = dynamic_cast<FreeGraph*>(this);
    if (asUniform)
        asUniform->WriteToStream(file);
    if (asFree)
        asFree->WriteToStream(file);

    file.close();
}

void Graph::WriteToDisk(const std::string& fileName, Description desc) {
    WriteToDisk(fileName, GetDescriptionName(desc));
}

void Graph::AddStats(json& stats) const {
    stats["amounts"] = {{"vertices", GetVerticesConst().size()}, {"edges", GetEdgesConst().size()}};

    size_t numEdges = GetEdgesConst().size();
    size_t numEdgesMoreThanOnce = 0;
    for (auto& [id, edge] : GetEdgesConst()) {
        if (edge.data.samples > 1)
            ++numEdgesMoreThanOnce;
    }
    float moreThanOnceRatio = static_cast<float>(numEdgesMoreThanOnce) / static_cast<float>(numEdges);
    stats["edges"] = {{"more_than_once", numEdgesMoreThanOnce}, {"more_than_once_ratio", moreThanOnceRatio}};

    {
        auto [avg, std, var] = inNodePathLengthAverager.GetInfo();
        stats["in_node_path_length"] = {{"avg", avg}, {"std", std}};
    }
    {
        util::Averager searchRangeAverager(GetVerticesConst().size());
        for (auto& [id, vertex] : GetVerticesConst())
            searchRangeAverager.AddValue(vertex.data.renderSearchRange);
        auto [avg, std, var] = searchRangeAverager.GetInfo();
        stats["render_search_range"] = {{"avg", avg}, {"std", std}};
    }
}

void Graph::CheckSequentialIds() const {
    std::vector<int> ids;
    ids.reserve(vertices.size());

    for (auto& [id, _] : vertices)
        ids.push_back(id);

    std::sort(ids.begin(), ids.end(), [](int a, int b) { return a < b; });

    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] != i)
            ErrorExit("Graph must have sequential vertex ids");
    }
}

util::VerticesHolder Graph::GetVerticesList() const {
    CheckSequentialIds();

    std::vector<Point3f> vList;
    vList.reserve(vertices.size());

    for (int id = 0; id < vertices.size(); ++id) {
        const Vertex& vertex = GetVertexConst(id)->get();
        vList.push_back(vertex.point);
    }

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
        // TODO: merge vertex data
        return GetVertex(findResult->second).value();
    }

    auto emplaceResult = vertices.emplace(id, Vertex{id, coors * spacing, data, coors});

    Vertex& newVertex = emplaceResult.first->second;
    coorsMap.insert({coors, newVertex.id});

    if (incrId)
        ++curVertexId;

    return newVertex;
}

void UniformGraph::WriteToStream(std::ostream& out) const {
    out << "uniform" << SEP << spacing << NEW;

    Graph::WriteToStream(out);
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
    file.close();
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
        Vertex& curNewVertex = uniform.AddVertex(oldVertex.point, oldVertex.data);

        for (auto [otherOldVertexId, edgeId] : oldVertex.inEdges) {
            const Edge& oldEdge = GetEdgeConst(edgeId).value();

            if (otherOldVertexId == oldVertex.id) {
                uniform.AddEdge(curNewVertex.id, curNewVertex.id, oldEdge.data);
            } else {
                const Vertex& otherOldVertex = vertices.at(otherOldVertexId);
                Vertex& otherNewVertex = uniform.AddVertex(otherOldVertex.point, otherOldVertex.data);
                uniform.AddEdge(otherNewVertex.id, curNewVertex.id, oldEdge.data);
            }
        }

        for (auto [otherOldVertexId, edgeId] : oldVertex.outEdges) {
            const Edge& oldEdge = GetEdgeConst(edgeId).value();

            if (otherOldVertexId == oldVertex.id) {
                uniform.AddEdge(curNewVertex.id, curNewVertex.id, oldEdge.data);
            } else {
                const Vertex& otherOldVertex = vertices.at(otherOldVertexId);
                Vertex& otherNewVertex = uniform.AddVertex(otherOldVertex.point, otherOldVertex.data);
                uniform.AddEdge(curNewVertex.id, otherNewVertex.id, oldEdge.data);
            }
        }

        progress.Update();
    }

    for (auto& [id, oldPath]: paths) {
        Path& newPath = uniform.AddPath(oldPath.data);
        for (auto vertexId : oldPath.vertices) {
            uniform.AddVertexToPath(vertexId, newPath.id);
        }
    }

    progress.Done();

    return uniform;
}

void FreeGraph::WriteToStream(std::ostream& out) const {
    out << "free" << SEP << vertexRadius << NEW;

    Graph::WriteToStream(out);
}

void FreeGraph::ReadFromStream(std::istream& in) {
    std::string description, name;
    in >> description >> name >> vertexRadius;

    Graph::ReadFromStream(in);
}

FreeGraph FreeGraph::ReadFromDisk(const std::string& fileName) {
    FreeGraph graph;
    std::ifstream file(util::FileNameToPath(fileName));
    graph.ReadFromStream(file);
    file.close();
    return graph;
}

}
