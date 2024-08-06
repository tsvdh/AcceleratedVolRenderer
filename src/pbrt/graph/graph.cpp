#include "graph.h"
#include <fstream>
#include <stdexcept>

namespace graph {

char SEP(' ');
char NEW('\n');

// Helper methods
template<class T>
std::ostream& operator<<(std::ostream& out, Point3<T>& p) {
    out << p.x << SEP << p.y << SEP << p.z << SEP;
    return out;
}

template<class T>
std::istream& operator>>(std::istream& in, Point3<T>& p) {
    in >> p.x >> p.y >> p.z;
    return in;
}

// Graph implementations
template<typename T>
std::optional<T*> GetById(int id, std::unordered_map<int, T*> coll) {
    auto result = coll.find(id);
    if (result == coll.end())
        return {};

    return result->second;
}

std::optional<Vertex*> Graph::GetVertex(int id) { return GetById(id, vertices); }
std::optional<Edge*> Graph::GetEdge(int id) { return GetById(id, edges); }
std::optional<Path*> Graph::GetPath(int id) { return GetById(id, paths); }

bool Graph::RemoveVertex(int id) {
    auto result = vertices.find(id);
    if (result == vertices.end())
        return false;

    for (Edge* edge : result->second->inEdges)
        RemoveEdge(edge->id);

    for (Edge* edge : result->second->outEdges)
        RemoveEdge(edge->id);

    delete result->second;
    vertices.erase(id);
    return true;
}

std::optional<Edge*> Graph::AddEdge(Vertex* from, Vertex* to, EdgeData* data, bool checkValid) {
    if (checkValid) {
        if (vertices.find(from->id) == vertices.end())
            return {};
        if (vertices.find(to->id) == vertices.end())
            return {};
    }

    for (Edge* edge: from->outEdges) {
        if (*edge->to == *to)
            // TODO: merge edge data
            throw std::runtime_error("Can't merge edges yet");
    }

    auto newEdge = new Edge{++curId, from, to, data};
    edges[curId] = newEdge;

    from->outEdges.insert(newEdge);
    to->inEdges.insert(newEdge);

    return newEdge;
}

std::optional<Edge*> Graph::AddEdge(int id, int fromId, int toId, EdgeData* data) {
    auto fromResult = vertices.find(fromId);
    if (fromResult == vertices.end())
        return {};

    auto toResult = vertices.find(toId);
    if (toResult == vertices.end())
        return {};

    Vertex* from = fromResult->second;
    Vertex* to = toResult->second;

    for (Edge* edge: from->outEdges) {
        if (*edge->to == *to)
            // TODO: merge edge data
            throw std::runtime_error("Can't merge edges yet");
    }

    auto newEdge = new Edge{id, from, to, data};
    edges[id] = newEdge;

    from->outEdges.insert(newEdge);
    to->inEdges.insert(newEdge);

    return newEdge;
}

bool Graph::RemoveEdge(int id) {
    auto result = edges.find(id);
    if (result == edges.end())
        return false;

    Edge* edge = result->second;

    edge->from->outEdges.erase(edge);
    edge->to->inEdges.erase(edge);

    for (auto [path, index] : edge->paths)
        paths[path->id]->edges[index] = nullptr;

    delete edge;
    edges.erase(id);
    return true;
}

void Graph::AddPath(const Path& path) {
    auto newPath = new Path{++curId};
    paths[curId] = newPath;

    if (path.edges.empty())
        return;

    Vertex* firstVertex = path.edges[0]->from;
    Vertex* curFrom = AddVertex(firstVertex->point, firstVertex->data);

    for (int i = 0; i < path.edges.size(); ++i) {
        Edge* edge = path.edges[i];
        Vertex* curTo = AddVertex(edge->to->point, edge->to->data);
        Edge* newEdge = AddEdge(curFrom, curTo, edge->data, true).value();
        AddEdgeToPath(newEdge, path.edgeData[i], newPath);

        curFrom = curTo;
    }
}

Path* Graph::AddPath() {
    auto path = new Path{++curId};
    paths[curId] = path;
    return path;
}

bool Graph::RemovePath(int id) {
    auto result = paths.find(id);
    if (result == paths.end())
        return false;

    Path* path = result->second;
    if (!path->edges.empty()) {
        for (Edge* edge : path->edges) {
            edge->paths.erase(path);

            if (edge->paths.empty()) {
                RemoveEdge(edge->id);

                if (Vertex* to = edge->to; to->inEdges.empty() && to->outEdges.empty())
                    RemoveVertex(to->id);
            }
        }

        Vertex* firstVertex = path->edges[0]->from;
        if (firstVertex->inEdges.empty() && firstVertex->outEdges.size() == 1)
            RemoveVertex(firstVertex->id);
    }

    delete result->second;
    paths.erase(id);
    return true;
}

bool Graph::AddEdgeToPath(Edge* edge, EdgeData* data, Path* path) {
    for (Edge* pathEdge : path->edges) {
        if (*pathEdge == *edge)
            return false;
    }

    path->edges.push_back(edge);
    path->edgeData.push_back(data);
    int index = static_cast<int>(path->edges.size());
    edge->paths[path] = index;

    return true;
}

inline void WriteEdgeData(std::ostream& out, EdgeData* data) {
    for (int i = 0; i < NSpectrumSamples; ++i) {
        out << data->throughput[i] << SEP;
    }
    out << data->weightedThroughput << SEP;
}

inline EdgeData* ReadEdgeData(std::istream& in) {
    std::vector<float> throughputs(NSpectrumSamples);
    for (int i = 0; i < NSpectrumSamples; ++i)
        in >> throughputs[i];

    float weightedThroughput;
    in >> weightedThroughput;

    return new EdgeData{SampledSpectrum(throughputs), weightedThroughput};
}

void Graph::WriteToStream(std::ostream& out, StreamFlags flags) {
    out << (flags.useCoors ? "True" : "False") << SEP
        << (flags.useThroughput ? "True" : "False") << SEP
        << (flags.useRayVertexTypes ? "True" : "False") << NEW;

    out << curId << SEP <<
           vertices.size() << SEP <<
           edges.size() << SEP <<
           paths.size() << NEW;

    for (auto [_, vertex] : vertices) {
        out << vertex->id << SEP << vertex->point;

        if (flags.useRayVertexTypes) {
            std::optional<RayVertexType> vertexType = vertex->data->type;
            out << (vertexType.has_value() ? vertexType.value() : -1) << SEP;
        }

        if (flags.useCoors)
            out << vertex->coors.value();
        out << NEW;
    }

    for (auto [_, edge] : edges) {
        out << edge->id << SEP << edge->from->id << SEP << edge->to->id << SEP;
        if (flags.useThroughput)
            WriteEdgeData(out, edge->data);
        out << NEW;
    }

    for (auto [_, path] : paths) {
        out << path->id << SEP << path->edges.size() << SEP;
        for (int i = 0; i < path->edges.size(); ++i) {
            out << path->edges[i]->id << SEP;
            if (flags.useThroughput)
                WriteEdgeData(out, path->edgeData[i]);
        }
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

        auto vertex = AddVertex(id, point, new VertexData{rayType});

        if (flags.useCoors) {
            Point3i coors;
            in >> coors;
            vertex->coors = coors;
        }
    }

    for (int i = 0; i < edgesCap; ++i) {
        int id, fromId, toId;
        in >> id >> fromId >> toId;
        EdgeData* data = flags.useThroughput ? ReadEdgeData(in) : nullptr;

        AddEdge(id, fromId, toId, data);
    }

    for (int i = 0; i < pathsCap; ++i) {
        int id, pathLength;
        in >> id >> pathLength;

        auto path = new Path{id};
        paths[id] = path;

        for (int j = 0; j < pathLength; ++j) {
            int pathEdgeId;
            in >> pathEdgeId;
            EdgeData* data = flags.useThroughput ? ReadEdgeData(in) : nullptr;

            AddEdgeToPath(edges[pathEdgeId], data, path);
        }
    }
}

void Graph::WriteToDisk(const std::string& fileName, const std::string& desc, StreamFlags flags) {
    std::ofstream file("files/graphs/" + fileName + ".txt");
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

// UniformGraph implementations
std::optional<Vertex*> UniformGraph::GetVertex(Point3i coors) {
    auto result = coorsMap.find(coors);
    if (result == coorsMap.end())
        return {};

    return result->second;
}

Vertex* UniformGraph::AddVertex(Point3f p, VertexData* data) {
    auto [coors, fittedPoint] = FitToGraph(p);
    auto newVertex = new Vertex{curId + 1, fittedPoint, data, coors};

    auto [iter, success] = coorsMap.insert({coors, newVertex});
    if (success) {
        vertices[++curId] = newVertex;
        return newVertex;
    }
    else {
        // TODO: merge vertex data
        delete newVertex;
        return iter->second;
    }
}

Vertex* UniformGraph::AddVertex(int id, Point3f p, VertexData* data) {
    auto [coors, fittedPoint] = FitToGraph(p);
    auto newVertex = new Vertex{id, fittedPoint, data, coors};

    auto [iter, success] = coorsMap.insert({coors, newVertex});
    if (success) {
        vertices[id] = newVertex;
        return newVertex;
    }
    return iter->second;
}

Vertex* UniformGraph::AddVertex(Point3i coors, VertexData* data) {
    return AddVertex(Point3f(coors * spacing), data);
}

bool UniformGraph::RemoveVertex(int id) {
    auto result = vertices.find(id);
    if (result == vertices.end())
        return false;

    coorsMap.erase(result->second->coors.value());

    return Graph::RemoveVertex(id);
}

inline std::tuple<Point3i, Point3f> UniformGraph::FitToGraph(const Point3f& p) const {
    Point3i coors;
    for (int i = 0; i < 3; ++i) {
        coors[i] = static_cast<int>(std::round(p[i] / spacing));
    }

    Point3f fittedPoint = coors * spacing;

    return {coors, fittedPoint};
}

void UniformGraph::WriteToStream(std::ostream& out, StreamFlags flags) {
    out << "uniform" << SEP << spacing << NEW;

    Graph::WriteToStream(out, flags);
}

void UniformGraph::ReadFromStream(std::istream& in) {
    std::string description, name;
    in >> description >> name >> spacing;

    Graph::ReadFromStream(in);
}

UniformGraph* UniformGraph::ReadFromDisk(const std::string& fileName) {
    auto graph = new UniformGraph;
    std::ifstream file(FileNameToPath(fileName));
    graph->ReadFromStream(file);
    return graph;
}

// FreeGraph implementations
UniformGraph* FreeGraph::ToUniform(float spacing) {
    auto uniform = new UniformGraph(spacing);

    for (auto [oldId, oldVertex] : vertices) {
        Vertex* curVertex = uniform->AddVertex(oldVertex->point, oldVertex->data);

        for (Edge* inEdge : oldVertex->inEdges) {
            Vertex* inVertex = uniform->AddVertex(inEdge->from->point, inEdge->from->data);
            uniform->AddEdge(inVertex, curVertex, inEdge->data, false);
        }

        for (Edge* outEdge : oldVertex->outEdges) {
            Vertex* outVertex = uniform->AddVertex(outEdge->to->point, outEdge->to->data);
            uniform->AddEdge(curVertex, outVertex, outEdge->data, false);
        }
    }

    for (auto [oldId, oldPath] : paths) {
        auto newPath = AddPath();
        for (int i = 0; i < oldPath->edges.size(); ++i) {
            AddEdgeToPath(oldPath->edges[i], oldPath->edgeData[i], newPath);
        }
    }

    return uniform;
}

void FreeGraph::WriteToStream(std::ostream& out, StreamFlags flags) {
    out << "free" << NEW;

    Graph::WriteToStream(out, flags);
}

void FreeGraph::ReadFromStream(std::istream& in) {
    std::string description, name;
    in >> description >> name;

    Graph::ReadFromStream(in);
}

FreeGraph* FreeGraph::ReadFromDisk(const std::string& fileName) {
    auto graph = new FreeGraph;
    std::ifstream file(FileNameToPath(fileName));
    graph->ReadFromStream(file);
    return graph;
}

}
