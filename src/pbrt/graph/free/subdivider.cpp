#include "subdivider.h"

#include "free_graph_builder.h"
#include "free_lighting_calculator.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

Subdivider::Subdivider(Graph& graph, const util::MediumData& mediumData, Vector3f lightDir, Sampler sampler, float baseRadius, SubDividerConfig config)
        : graph(graph), mediumData(mediumData), lightDir(lightDir), sampler(std::move(sampler)), baseRadius(baseRadius), config(std::move(config)) {
    sphereInterface = MediumInterface(mediumData.medium);
}

inline Sphere MakeSphere(float radius) {
    return Sphere(nullptr, nullptr, false, radius, -radius, radius, 360);
}

void Subdivider::ComputeSubdivisionEffect(SparseVec& initialLight) {
    if (config.subdivisions == 0)
        return;

    if (config.subdivisions > 1)
        ErrorExit("Number of subdivisions limited to 1");

    int workNeeded = initialLight.size();
    for (auto& [_, vertex] : graph.GetVertices())
        workNeeded += vertex.inEdges.size();

    Sphere vertexSphere = MakeSphere(baseRadius);
    float graphRadius = baseRadius * config.graphBuilder.radiusModifier;

    ProgressReporter progress(workNeeded, "Computing subdivision effect", false);

    for (int id = 0; id < graph.GetVertices().size(); ++id) {
        Vertex& vertex = graph.GetVertex(id)->get();

        Transform vertexTransform = Translate(Point3f(0, 0, 0) - vertex.point);
        vertexSphere.SetObjectFromRender(&vertexTransform);

        if (initialLight.coeff(id) != 0) {
            initialLight.coeffRef(id) *= Subdivide(vertexSphere, lightDir, graphRadius);
            progress.Update();
        }

        for (auto& [otherId, edgeId] : vertex.inEdges) {
            Edge& edge = graph.GetEdge(edgeId)->get();
            Vertex& otherVertex = graph.GetVertex(otherId)->get();
            Vector3f edgeDir = vertex.point - otherVertex.point;

            edge.data.throughput *= Subdivide(vertexSphere, edgeDir, graphRadius);
            progress.Update();
        }
    }
    progress.Done();
}

float Subdivider::Subdivide(const Sphere& sphere, Vector3f inDirection, float graphRadius) const {
    GeometricPrimitive primitive(&sphere, nullptr, nullptr, sphereInterface);
    util::PrimitiveData primitiveData(&primitive);
    util::MediumData localMediumData = mediumData;
    localMediumData.primitiveData = primitiveData;

    FreeGraphBuilder graphBuilder(localMediumData, inDirection, sampler, config.graphBuilder, graphRadius, true);
    FreeGraph graph = graphBuilder.TracePaths();
    graphBuilder.ComputeTransmittance(graph);

    FreeLightingCalculator lightingCalculator(graph, localMediumData, inDirection, sampler, config.lightingCalculator, true);
    lightingCalculator.ComputeFinalLight();

    float averageLight = 0;
    for (auto& [id, vertex] : graph.GetVertices())
        averageLight += vertex.data.lightScalar;

    return averageLight / static_cast<float>(graph.GetVertices().size());
}

}
