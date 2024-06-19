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
std::optional<Vertex*> Graph::GetVertex(int id) {
    auto result = vertices.find(id);
    if (result == vertices.end())
        return {};

    return result->second;
}

std::optional<Edge*> Graph::GetEdge(int id) {
    auto result = edges.find(id);
    if (result == edges.end())
        return {};

    return result->second;
}

std::optional<Edge*> Graph::AddEdge(graph::Vertex* from, graph::Vertex* to, graph::EdgeData* data, bool checkValid) {
    if (checkValid) {
        if (vertices.find(from->id) == vertices.end())
            return {};
        if (vertices.find(to->id) == vertices.end())
            return {};
    }

    auto newEdge = new Edge{++curId, from, to, data};
    edges.insert({curId, newEdge});

    from->outEdges.push_back(newEdge);
    to->inEdges.push_back(newEdge);

    return newEdge;
}

std::optional<Edge*> Graph::AddEdge(int id, int fromId, int toId, graph::EdgeData* data) {
    auto fromResult = vertices.find(fromId);
    if (fromResult == vertices.end())
        return {};

    auto toResult = vertices.find(toId);
    if (toResult == vertices.end())
        return {};

    auto newEdge = new Edge{id, fromResult->second, toResult->second, data};
    edges[id] = newEdge;

    fromResult->second->outEdges.push_back(newEdge);
    toResult->second->inEdges.push_back(newEdge);

    return newEdge;
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
        newPath->edges.push_back(newEdge);

        curFrom = curTo;
    }
}

Path* Graph::AddPath() {
    auto path = new Path{++curId};
    paths[curId] = path;
    return path;
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
        auto vertex = new Vertex{id, point};
        vertices[id] = vertex;

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

            path->edges.push_back(edges[pathEdgeId]);
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

// UniformGraph implementations
Vertex* UniformGraph::AddVertex(pbrt::Point3f p) {
    auto [coors, fittedPoint] = FitToGraph(p);
    auto newVertex = new Vertex{curId + 1, fittedPoint, coors};

    auto result = coorsMap.insert({coors.ToString(), newVertex});
    if (result.second) {
        vertices[++curId] = newVertex;
        return newVertex;
    }
    return result.first->second;
}

Vertex* UniformGraph::AddVertex(int id, pbrt::Point3f p) {
    auto [coors, fittedPoint] = FitToGraph(p);
    auto newVertex = new Vertex{id, fittedPoint, coors};

    auto result = coorsMap.insert({coors.ToString(), newVertex});
    if (result.second) {
        vertices[id] = newVertex;
        return newVertex;
    }
    return result.first->second;
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
    std::string name;
    in >> name >> g.spacing;

    g.ReadFromStream(in);
    return in;
}

// FreeGraph implementations
UniformGraph* FreeGraph::ToUniform(float spacing) {
    auto uniform = new UniformGraph(spacing);

    for (auto oldPair : vertices) {
        Vertex* oldVertex = oldPair.second;
        Vertex* curVertex = uniform->AddVertex(oldVertex->point);

        for (Edge* inEdge : oldVertex->inEdges) {
            Vertex* inVertex = uniform->AddVertex(inEdge->from->point);
            uniform->AddEdge(inVertex, curVertex, inEdge->data, false);
        }

        for (Edge* outEdge : oldVertex->outEdges) {
            Vertex* outVertex = uniform->AddVertex(outEdge->to->point);
            uniform->AddEdge(curVertex, outVertex, outEdge->data, false);
        }
    }

    // TODO: maintain paths

    return uniform;
}

std::ostream& operator<<(std::ostream& out, FreeGraph& g) {
    out << "free" << NEW;

    g.WriteToStream(out, StreamFlags{false});
    return out;
}

std::istream& operator>>(std::istream& in, FreeGraph& g) {
    std::string name;
    in >> name;

    g.ReadFromStream(in);
    return in;
}

}
