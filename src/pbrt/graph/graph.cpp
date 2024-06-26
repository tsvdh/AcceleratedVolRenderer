#include "graph.h"
#include <stdexcept>
#include <iostream>

namespace graph {

char SEP(' ');
char NEW('\n');

// Helper methods
template<class T>
std::ostream& operator<<(std::ostream& out, pbrt::Point3<T>& p) {
    out << p.x << SEP << p.y << SEP << p.z << SEP;
    return out;
}

template<class T>
std::istream& operator>>(std::istream& in, pbrt::Point3<T>& p) {
    in >> p.x >> p.y >> p.z;
    return in;
}

// Graph implementations
template<typename T>
inline std::optional<T*> GetById(int id, std::unordered_map<int, T*> coll) {
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

    for (auto pair : result->second->inEdges)
        RemoveEdge(pair.second->id);

    for (auto pair : result->second->outEdges)
        RemoveEdge(pair.second->id);

    delete result->second;
    vertices.erase(id);
    return true;
}

std::optional<Edge*> Graph::AddEdge(graph::Vertex* from, graph::Vertex* to, graph::EdgeData* data, bool checkValid) {
    if (checkValid) {
        if (vertices.find(from->id) == vertices.end())
            return {};
        if (vertices.find(to->id) == vertices.end())
            return {};
    }

    for (auto pair: from->outEdges) {
        if (*pair.second->to == *to)
            // TODO: merge edge data
            return pair.second;
    }

    auto newEdge = new Edge{++curId, from, to, data};
    edges[curId] = newEdge;

    from->outEdges[curId] = newEdge;
    to->inEdges[curId] = newEdge;

    return newEdge;
}

std::optional<Edge*> Graph::AddEdge(int id, int fromId, int toId, graph::EdgeData* data) {
    auto fromResult = vertices.find(fromId);
    if (fromResult == vertices.end())
        return {};

    auto toResult = vertices.find(toId);
    if (toResult == vertices.end())
        return {};

    for (auto pair : fromResult->second->outEdges) {
        if (*pair.second->to == *toResult->second)
            // TODO: merge edge data
            return pair.second;
    }

    auto newEdge = new Edge{id, fromResult->second, toResult->second, data};
    edges[id] = newEdge;

    fromResult->second->outEdges[id] = newEdge;
    toResult->second->inEdges[id] = newEdge;

    return newEdge;
}

bool Graph::RemoveEdge(int id) {
    auto result = edges.find(id);
    if (result == edges.end())
        return false;

    result->second->from->outEdges.erase(id);
    result->second->to->inEdges.erase(id);

    for (auto pair : result->second->paths)
        paths[pair.first]->edges[pair.second] = nullptr;

    delete result->second;
    edges.erase(id);
    return true;
}

void Graph::AddPath(const graph::Path& path) {
    auto newPath = new Path{++curId};
    paths[curId] = newPath;

    if (path.edges.empty())
        return;

    Vertex* curFrom = AddVertex(path.edges[0]->from->point);
    Vertex* curTo;

    for (Edge* edge : path.edges) {
        curTo = AddVertex(edge->to->point);
        Edge* newEdge = AddEdge(curFrom, curTo, edge->data, true).value();
        AddEdgeToPath(newEdge, newPath);

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
            edge->paths.erase(id);

            if (edge->paths.empty()) {
                RemoveEdge(edge->id);

                Vertex* to = edge->to;
                if (to->inEdges.empty() && to->outEdges.empty())
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

bool Graph::AddEdgeToPath(graph::Edge* edge, graph::Path* path) {
    for (Edge* pathEdge : path->edges) {
        if (*pathEdge == *edge)
            return false;
    }

    path->edges.push_back(edge);
    int index = (int)path->edges.size();
    edge->paths[path->id] = index;

    return true;
}

void Graph::WriteToStream(std::ostream& out, StreamFlags flags) {
    out << (flags.useCoors ? "True" : "False") << NEW;

    out << curId << SEP <<
           vertices.size() << SEP <<
           edges.size() << SEP <<
           paths.size() << NEW;

    for (auto pair : vertices) {
        Vertex* vertex = pair.second;
        out << vertex->id << SEP << vertex->point ;
        if (flags.useCoors)
            out << vertex->coors.value();
        out << NEW;
    }

    for (auto pair : edges) {
        Edge* edge = pair.second;
        out << edge->id << SEP << edge->from->id << SEP << edge->to->id << NEW;
    }

    for (auto pair : paths) {
        Path* path = pair.second;
        out << path->id << SEP << path->edges.size() << SEP;
        for (Edge* edge : path->edges) {
            out << edge->id << SEP;
        }
        out << NEW;
    }
}

void Graph::ReadFromStream(std::istream& in) {
    StreamFlags flags{};

    std::string useCoors;
    in >> useCoors;
    if (useCoors == "True")
        flags.useCoors = true;

    int verticesCap, edgesCap, pathsCap;
    in >> curId >> verticesCap >> edgesCap >> pathsCap;
    vertices.reserve(verticesCap);
    edges.reserve(edgesCap);
    paths.reserve(pathsCap);

    for (int i = 0; i < verticesCap; ++i) {
        int id;
        pbrt::Point3f point;
        in >> id >> point;
        auto vertex = AddVertex(id, point);

        if (flags.useCoors) {
            pbrt::Point3i coors;
            in >> coors;
            vertex->coors = coors;
        }
    }

    for (int i = 0; i < edgesCap; ++i) {
        int id, fromId, toId;
        in >> id >> fromId >> toId;

        AddEdge(id, fromId, toId, nullptr);
    }

    for (int i = 0; i < pathsCap; ++i) {
        int id, pathLength;
        in >> id >> pathLength;

        auto path = new Path{id};
        paths[id] = path;

        for (int j = 0; j < pathLength; ++j) {
            int pathEdgeId;
            in >> pathEdgeId;

            AddEdgeToPath(edges[pathEdgeId], path);
        }
    }
}

void Graph::WriteToDisk(const std::string& fileName, const std::string& desc) {
    std::ofstream file("files/graphs/" + fileName + ".txt");
    file << desc << NEW;

    auto asUniform = dynamic_cast<UniformGraph*>(this);
    auto asFree = dynamic_cast<FreeGraph*>(this);
    if (asUniform)
        file << *asUniform;
    if (asFree)
        file << *asFree;

    file.close();
}

void Graph::WriteToDisk(const std::string& fileName, graph::Description desc) {
    WriteToDisk(fileName, GetDescriptionName(desc));
}

// UniformGraph implementations
std::optional<Vertex*> UniformGraph::GetVertex(pbrt::Point3i coors) {
    auto result = coorsMap.find(coors);
    if (result == coorsMap.end())
        return {};

    return result->second;
}

Vertex* UniformGraph::AddVertex(pbrt::Point3f p) {
    auto [coors, fittedPoint] = FitToGraph(p);
    auto newVertex = new Vertex{curId + 1, fittedPoint, coors};

    auto result = coorsMap.insert({coors, newVertex});
    if (result.second) {
        vertices[++curId] = newVertex;
        return newVertex;
    }
    return result.first->second;
}

Vertex* UniformGraph::AddVertex(int id, pbrt::Point3f p) {
    auto [coors, fittedPoint] = FitToGraph(p);
    auto newVertex = new Vertex{id, fittedPoint, coors};

    auto result = coorsMap.insert({coors, newVertex});
    if (result.second) {
        vertices[id] = newVertex;
        return newVertex;
    }
    return result.first->second;
}

Vertex* UniformGraph::AddVertex(pbrt::Point3i coors) {
    return AddVertex(Point3f(coors * spacing));
}

bool UniformGraph::RemoveVertex(int id) {
    auto result = vertices.find(id);
    if (result == vertices.end())
        return false;

    coorsMap.erase(result->second->coors.value());

    return Graph::RemoveVertex(id);
}

inline std::tuple<pbrt::Point3i, pbrt::Point3f> UniformGraph::FitToGraph(const pbrt::Point3f& p) const {
    pbrt::Point3i coors;
    pbrt::Point3f fittedPoint;

    for (int i = 0; i < 3; ++i) {
        coors[i] = (int) std::round(p[i] / spacing);
    }
    fittedPoint = coors * spacing;

    return {coors, fittedPoint};
}

std::ostream& operator<<(std::ostream& out, UniformGraph& g) {
    out << "uniform" << SEP << g.spacing << NEW;

    g.WriteToStream(out, StreamFlags{false});
    return out;
}

std::istream& operator>>(std::istream& in, UniformGraph& g) {
    std::string description, name;
    in >> description >> name >> g.spacing;

    g.ReadFromStream(in);
    return in;
}

UniformGraph* UniformGraph::ReadFromDisk(const std::string& fileName) {
    auto graph = new UniformGraph;
    std::ifstream file(graph::FileNameToPath(fileName));
    file >> *graph;
    return graph;
}

// FreeGraph implementations
UniformGraph* FreeGraph::ToUniform(float spacing) {
    auto uniform = new UniformGraph(spacing);

    for (auto oldPair : vertices) {
        Vertex* oldVertex = oldPair.second;
        Vertex* curVertex = uniform->AddVertex(oldVertex->point);

        for (auto pair : oldVertex->inEdges) {
            Edge* inEdge = pair.second;
            Vertex* inVertex = uniform->AddVertex(inEdge->from->point);
            uniform->AddEdge(inVertex, curVertex, inEdge->data, false);
        }

        for (auto pair : oldVertex->outEdges) {
            Edge* outEdge = pair.second;
            Vertex* outVertex = uniform->AddVertex(outEdge->to->point);
            uniform->AddEdge(curVertex, outVertex, outEdge->data, false);
        }
    }

    for (auto oldPair : paths) {
        Path* oldPath = oldPair.second;
        auto newPath = AddPath();
        for (Edge* edge : oldPath->edges)
            AddEdgeToPath(edge, newPath);
    }

    return uniform;
}

std::ostream& operator<<(std::ostream& out, FreeGraph& g) {
    out << "free" << NEW;

    g.WriteToStream(out, StreamFlags{false});
    return out;
}

std::istream& operator>>(std::istream& in, FreeGraph& g) {
    std::string description, name;
    in >> description >> name;

    g.ReadFromStream(in);
    return in;
}

FreeGraph* FreeGraph::ReadFromDisk(const std::string& fileName) {
    auto graph = new FreeGraph;
    std::ifstream file(graph::FileNameToPath(fileName));
    file >> *graph;
    return graph;
}

}
