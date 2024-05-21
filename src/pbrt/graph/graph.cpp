#include <pbrt/graph/graph.h>
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
    for (auto v: vertices) {
        if (v->id == id)
            return v;
    }
    return {};
}

std::optional<Edge*> Graph::GetEdge(int id) {
    for (auto& e : edges) {
        if (e->id == id)
            return e;
    }
    return {};
}

std::optional<Edge*> Graph::AddEdge(graph::Vertex* from, graph::Vertex* to, graph::EdgeData* data, bool checkValid) {
    if (checkValid) {
        bool hasFrom, hasTo;

        for (auto v: vertices) {
            if (v == from)
                hasFrom = true;
            if (v == to)
                hasTo = true;
        }

        if (!hasFrom || !hasTo)
            return {};
    }

    auto newEdge = new Edge{curId++, from, to, data};
    edges.push_back(newEdge);

    from->outEdges.push_back(newEdge);
    to->inEdges.push_back(newEdge);

    return newEdge;
}

std::optional<Edge*> Graph::AddEdge(int id, int fromId, int toId, graph::EdgeData* data) {
    Vertex* from, *to;
    bool hasFrom, hasTo;

    for (auto v : vertices) {
        if (v->id == fromId) {
            hasFrom = true;
            from = v;
        }
        if (v->id == toId) {
            hasTo = true;
            to = v;
        }
    }

    if (!hasFrom || !hasTo)
        return {};

    auto newEdge = new Edge{id, from, to, data};
    edges.push_back(newEdge);

    from->outEdges.push_back(newEdge);
    to->inEdges.push_back(newEdge);

    return newEdge;
}

void Graph::AddPath(graph::Path& path) {
    auto newPath = new Path{curId++};
    paths.push_back(newPath);

    if (path.edges.empty())
        return;

    Vertex* curFrom = AddVertex(path.edges[0]->from->point);
    Vertex* curTo;

    for (auto edge : path.edges) {
        curTo = AddVertex(edge->to->point);
        Edge* newEdge = AddEdge(curFrom, curTo, edge->data, false).value();
        newPath->edges.push_back(newEdge);

        curFrom = curTo;
    }
}

void Graph::WriteToStream(std::ostream& out, StreamFlags flags) {
    out << flags.useCoors << NEW;

    out << curId << SEP <<
           vertices.size() << SEP <<
           edges.size() << SEP <<
           paths.size() << NEW;

    for (auto vertex : vertices) {
        out << vertex->id << SEP << vertex->point ;
        if (flags.useCoors)
            out << vertex->coors.value();
        out << NEW;
    }

    for (auto edge : edges) {
        out << edge->id << SEP << edge->from->id << SEP << edge->to->id << NEW;
    }

    for (auto path : paths) {
        out << path->id << SEP << path->edges.size() << SEP;
        for (auto edge : path->edges) {
            out << edge->id << SEP;
        }
        out << NEW;
    }
}

void Graph::ReadFromStream(std::istream& in) {
    StreamFlags flags{};
    in >> flags.useCoors;

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
        vertices.push_back(vertex);

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
        paths.push_back(path);

        for (int j = 0; j < pathLength; ++j) {
            int pathId;
            in >> pathId;

            for (auto edge : edges) {
                if (edge->id == pathId) {
                    path->edges.push_back(edge);
                    break;
                }
            }
        }
    }
}

// UniformGraph implementations
Vertex* UniformGraph::AddVertex(pbrt::Point3f& p) {
    auto [coors, fittedPoint] = FitToGraph(p);

    for (auto vertex : vertices) {
        if (vertex->coors == coors)
            return vertex;
    }

    auto newVertex = new Vertex{curId++, fittedPoint, coors};
    vertices.push_back(newVertex);
    return newVertex;
}

inline std::tuple<pbrt::Point3i, pbrt::Point3f> UniformGraph::FitToGraph(pbrt::Point3f& p) const {
    pbrt::Point3i coors;
    pbrt::Point3f fittedPoint;

    for (int i = 0; i < 3; ++i) {
        coors[i] = (int) std::round(p[i] / spacing);
    }
    fittedPoint = coors * spacing;

    return {coors, fittedPoint};
}

std::ostream& operator<<(std::ostream& out, UniformGraph& g) {
    out << g.spacing << NEW;

    g.WriteToStream(out, StreamFlags{true});
    return out;
}

std::istream& operator>>(std::istream& in, UniformGraph& g) {
    in >> g.spacing;

    g.ReadFromStream(in);
    return in;
}

// FreeGraph implementations
UniformGraph* FreeGraph::ToUniform(float spacing) {
    auto uniform = new UniformGraph(spacing);

    for (auto oldVertex : vertices) {
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

    return uniform;
}

std::ostream& operator<<(std::ostream& out, FreeGraph& g) {
    g.WriteToStream(out, StreamFlags{false});
    return out;
}

std::istream& operator>>(std::istream& in, FreeGraph& g) {
    g.ReadFromStream(in);
    return in;
}

}
