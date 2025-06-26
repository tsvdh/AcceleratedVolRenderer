#pragma once

#include "pbrt/graph/graph.h"

#include "pbrt/graph/free/free_lighting_calculator.h"

using namespace pbrt;
using namespace graph;

SparseMat GetTransmittanceMatrix(const Graph& graph) {
    int numVertices = static_cast<int>(graph.GetVerticesConst().size());
    auto& edges = graph.GetEdgesConst();

    std::vector<Eigen::Triplet<float>> transmittanceEntries;
    transmittanceEntries.reserve(edges.size());

    for (auto& [id, edge] : edges) {
        float T = edge.data.throughput.value;
        transmittanceEntries.emplace_back(edge.to, edge.from, T);
    }

    SparseMat transmittanceMatrix(numVertices, numVertices);
    transmittanceMatrix.setFromTriplets(transmittanceEntries.begin(), transmittanceEntries.end());
    transmittanceMatrix = transmittanceMatrix.transpose();
    return transmittanceMatrix;
}

float Round(float value, float decimals) {
    float mult = static_cast<float>(std::pow(10, decimals));
    return std::round(value * mult) / mult;
}

void main(int argc, char* argv[]) {
    std::vector<SparseVec> lightVectors;

    FreeGraph graph;

    // one way
    // int numBounces = 4;
    // graph.AddVertex(0, Point3f(), VertexData{none, 0, {1, 1}});
    // graph.AddVertex(1, Point3f(), VertexData{none, 0, {0.5, 1}});
    // graph.AddVertex(2, Point3f(), VertexData{none, 0, {1, 1}});
    // graph.AddVertex(3, Point3f(), VertexData{none, 1, {1, 1}});
    // graph.AddVertex(4, Point3f(), VertexData{none, 1, {1, 1}});
    // graph.AddEdge(0, 0, 1, EdgeData{{1, 1}});
    // graph.AddEdge(1, 1, 2, EdgeData{{1, 1}});
    // graph.AddEdge(2, 2, 3, EdgeData{{1, 1}});
    // graph.AddEdge(3, 4, 2, EdgeData{{1, 1}});

    // // triangle
    int numBounces = 1;
    graph.AddVertex(0, Point3f(), VertexData{none, 0.3, {-1, 1}});
    graph.AddVertex(1, Point3f(), VertexData{none, 0.7, {2.f/3, 1}});
    graph.AddVertex(2, Point3f(), VertexData{none, 0.5, {1.f/2, 1}});
    graph.AddVertex(3, Point3f(), VertexData{none, .9, {2.f/5, 1}});
    graph.AddVertex(4, Point3f(), VertexData{none, .8, {-1, 1}});
    graph.AddEdge(0, 1, 0, EdgeData{{0.5, 1}});
    graph.AddEdge(1, 1, 3, EdgeData{{0.6, 1}});
    graph.AddEdge(2, 2, 1, EdgeData{{0.7, 1}});
    graph.AddEdge(3, 2, 3, EdgeData{{0.6, 1}});
    graph.AddEdge(4, 3, 1, EdgeData{{0.6, 1}});
    graph.AddEdge(5, 3, 4, EdgeData{{0.8, 1}});

    int numVertices = static_cast<int>(graph.GetVertices().size());

    SparseVec finalLight(numVertices);
    for (auto& [id, vertex] : graph.GetVertices())
        finalLight.coeffRef(id) = vertex.data.lightScalar;

    lightVectors.push_back(finalLight);

    SparseVec finalWeights(numVertices);
    for (int i = 0; i < numVertices; ++i)
        finalWeights.coeffRef(i) = 1.f;

    int curIteration = 0;

    SparseMat transmittance = GetTransmittanceMatrix(graph);
    SparseVec pathContinueVector(numVertices);
    for (auto& [id, vertex] : graph.GetVertices())
        pathContinueVector.coeffRef(id) = vertex.data.pathContinuePDF.value;

    SparseVec curLight = finalLight;
    SparseVec curWeights = finalWeights;

    for (; curIteration < numBounces; ++curIteration) {
        curLight = (transmittance * curLight).cwiseProduct(pathContinueVector);
        curWeights = transmittance * curWeights;

        bool invalidNumbers = false;
        for (int i = 0; i < numVertices; ++i) {
            if (IsNaN(curLight.coeff(i)) || IsInf(curLight.coeff(i))
                || IsNaN(curWeights.coeff(i)) || IsInf(curWeights.coeff(i))) {
                invalidNumbers = true;
                break;
                }
        }
        if (invalidNumbers)
            break;

        SparseVec invCurWeights(numVertices);
        for (int i = 0; i < numVertices; ++i) {
            float weight = curWeights.coeff(i);
            invCurWeights.coeffRef(i) = weight == 0.f ? 0.f : 1.f / weight;
        }
        finalLight += curLight.cwiseProduct(invCurWeights);

        lightVectors.push_back(finalLight);
    }

    int maxSegmentLength = static_cast<int>(std::string("- x bounces -").size());

    std::cout << "-";
    for (int bounces = 0; bounces <= numBounces; ++bounces) {
        std::cout << StringPrintf("- %s bounce%s -", bounces, bounces == 1 ? " " : "s");
    }
    std::cout << "-" << std::endl;

    std::cout << " ";
    for (int bounces = 0; bounces <= numBounces; ++bounces) {
        std::string labels("  id, L");
        for (int i = static_cast<int>(labels.size()); i < maxSegmentLength; ++i)
            labels.push_back(' ');
        std::cout << labels;
    }
    std::cout << std::endl;

    for (int id = 0; id < numVertices; ++id) {
        std::cout << " ";
        for (int bounces = 0; bounces <= numBounces; ++bounces) {
            std::string values = StringPrintf("  %s,  %s", id, Round(lightVectors[bounces].coeff(id), 2));
            for (int i = static_cast<int>(values.size()); i < maxSegmentLength; ++i)
                values.push_back(' ');
            std::cout << values;
        }
        std::cout << std::endl;
    }
}
