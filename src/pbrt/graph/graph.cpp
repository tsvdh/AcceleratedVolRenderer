#include <pbrt/graph/graph.h>
#include <stdexcept>

namespace graph {

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

std::optional<Edge*> Graph::AddEdge(Vertex* from, Vertex* to) {
    return AddEdge(from, to, nullptr);
}

std::optional<Edge*> Graph::AddEdge(graph::Vertex* from, graph::Vertex* to, graph::EdgeData* data) {
    bool hasFrom, hasTo;

    for (auto v : vertices) {
        if (v == from)
            hasFrom = true;
        if (v == to)
            hasTo = true;
    }

    if (!hasFrom || !hasTo)
        return {};

    auto newEdge = new Edge{curId++, from, to, data};
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
        Edge* newEdge = AddEdge(curFrom, curTo, edge->data).value();
        newPath->edges.push_back(newEdge);

        curFrom = curTo;
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

// FreeGraph implementations
UniformGraph* FreeGraph::ToUniform(float spacing) {
    auto uniform = new UniformGraph(spacing);

    for (auto oldVertex : vertices) {
        Vertex* curVertex = uniform->AddVertex(oldVertex->point);

        for (Edge* inEdge : oldVertex->inEdges) {
            Vertex* inVertex = uniform->AddVertex(inEdge->from->point);
            uniform->AddEdge(inVertex, curVertex, inEdge->data);
        }

        for (Edge* outEdge : oldVertex->outEdges) {
            Vertex* outVertex = uniform->AddVertex(outEdge->to->point);
            uniform->AddEdge(curVertex, outVertex, outEdge->data);
        }
    }

    return uniform;
}

}
