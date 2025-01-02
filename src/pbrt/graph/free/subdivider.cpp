#include "subdivider.h"

#include "free_graph_builder.h"
#include "free_lighting_calculator.h"
#include "pbrt/util/progressreporter.h"

namespace graph {
Subdivider::Subdivider(Graph& graph, const util::MediumData& mediumData, Vector3f lightDir, Sampler sampler, float baseRadius,
                       const SubDividerConfig& config, bool runInParallel)
    : graph(graph), mediumData(mediumData), lightDir(lightDir), sampler(std::move(sampler)), baseRadius(baseRadius), config(config),
      runInParallel(runInParallel) {
    sphereInterface = MediumInterface(mediumData.medium);
}

inline Sphere MakeSphere(float radius) {
    return Sphere(nullptr, nullptr, false, false,
                  radius, -radius, radius, 360);
}

void Subdivider::ComputeSubdivisionEffect(SparseVec& initialLight) {
    if (config.subdivisions == 0)
        return;
    if (config.subdivisions > 1)
        ErrorExit("Number of subdivisions limited to 1");

    int numVertices = static_cast<int>(graph.GetVertices().size());
    int workNeeded = 0;
    for (int id = 0; id < numVertices; ++id)
        if (initialLight.coeff(id) != 0)
            ++workNeeded;

    for (auto& [_, vertex] : graph.GetVertices())
        workNeeded += static_cast<int>(vertex.inEdges.size());

    Sphere defaultSphere = MakeSphere(baseRadius);
    float graphRadius = Sqr(std::sqrt(baseRadius) * config.graphBuilder.radiusModifier);

    ProgressReporter progress(workNeeded, "Computing subdivision effect", false);

    ParallelFor(0, numVertices, runInParallel, [&](int vertexId)  {
        Vertex& vertex = graph.GetVertex(vertexId)->get();

        Transform objectFromWorld = Translate(Point3f(0, 0, 0) - vertex.point);
        Transform worldFromObject = Inverse(objectFromWorld);

        Sphere currentSphere = defaultSphere;
        currentSphere.SetObjectFromRender(&objectFromWorld);
        currentSphere.SetRenderFromObject(&worldFromObject);

        if (initialLight.coeff(vertexId) != 0) {
            initialLight.coeffRef(vertexId) *= Subdivide(currentSphere, lightDir, graphRadius);
            progress.Update();
        }

        for (auto& [otherId, edgeId] : vertex.inEdges) {
            Edge& edge = graph.GetEdge(edgeId)->get();
            Vertex& otherVertex = graph.GetVertex(otherId)->get();
            Vector3f edgeDir = Normalize(vertex.point - otherVertex.point);

            edge.data.throughput *= Subdivide(currentSphere, edgeDir, graphRadius);

            progress.Update();
        }
    });
    progress.Done();
}

float Subdivider::Subdivide(const Sphere& sphere, Vector3f inDirection, float graphRadius) {
    GeometricPrimitive primitive(&sphere, nullptr, nullptr, sphereInterface);
    util::PrimitiveData primitiveData(&primitive);
    util::MediumData localMediumData = mediumData;
    localMediumData.primitiveData = primitiveData;

    FreeGraphBuilder graphBuilder(localMediumData, inDirection, sampler, config.graphBuilder, true, false, graphRadius);
    FreeGraph graph = graphBuilder.TracePaths();
    graphBuilder.ComputeTransmittance(graph);

    FreeLightingCalculator lightingCalculator(graph, localMediumData, inDirection, sampler, config.lightingCalculator, true, false);
    lightingCalculator.ComputeFinalLight();

    float averageLight = 0;
    for (auto& [id, vertex] : graph.GetVertices())
        averageLight += vertex.data.lightScalar;

    float numVertices = static_cast<float>(graph.GetVertices().size());
    return numVertices == 0 ? 1 : averageLight / numVertices;
}
}
