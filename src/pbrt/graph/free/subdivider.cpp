#include "subdivider.h"

#include "free_graph_builder.h"
#include "free_lighting_calculator.h"
#include "pbrt/util/progressreporter.h"

namespace graph {
Subdivider::Subdivider(FreeGraph& graph, const util::MediumData& mediumData, Vector3f lightDir, Sampler sampler, const SubDividerConfig& config)
    : graph(graph), mediumData(mediumData), lightDir(lightDir), sampler(std::move(sampler)), config(config) {
    if (!graph.GetVertexRadius())
        ErrorExit("Graph must have a vertex radius");
    sphereInterface = MediumInterface(mediumData.medium);
}

void Subdivider::ComputeSubdivisionEffect(SparseVec& initialLight) {
    if (config.subdivisions == 0)
        return;
    if (config.subdivisions > 1)
        ErrorExit("Number of subdivisions limited to 1");

    float totalLight = 0;
    int totalDuration = 0;
    int numSubdivisions = 0;

    int numVertices = static_cast<int>(graph.GetVertices().size());
    int64_t workNeeded = 0;
    for (int id = 0; id < numVertices; ++id)
        if (initialLight.coeff(id) != 0)
            ++workNeeded;

    for (auto& [_, vertex] : graph.GetVertices())
        workNeeded += static_cast<int>(vertex.inEdges.size());

    float baseSphereRadius = graph.GetVertexRadius().value();
    util::SphereMaker sphereMaker(baseSphereRadius);
    float squaredSubRadius = Sqr(baseSphereRadius * config.graphBuilder.radiusModifier);

    ProgressReporter progress(workNeeded, "Computing subdivision effect", false);

    ParallelFor(0, numVertices, config.runInParallel, [&](int vertexId)  {
        Vertex& vertex = graph.GetVertex(vertexId)->get();

        SphereContainer currentSphere = sphereMaker.GetSphereFor(vertex.point);

        if (initialLight.coeff(vertexId) != 0) {
            auto start = std::chrono::high_resolution_clock::now();
            float light = Subdivide(vertexId, currentSphere.sphere, lightDir, squaredSubRadius);
            auto end = std::chrono::high_resolution_clock::now();
            totalDuration += static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

            totalLight += light;
            ++numSubdivisions;
            initialLight.coeffRef(vertexId) *= light;
            progress.Update();
        }

        for (auto& [otherId, edgeId] : vertex.inEdges) {
            Edge& edge = graph.GetEdge(edgeId)->get();
            Vertex& otherVertex = graph.GetVertex(otherId)->get();
            Vector3f edgeDir = Normalize(vertex.point - otherVertex.point);

            auto start = std::chrono::high_resolution_clock::now();
            float light = Subdivide(vertexId, currentSphere.sphere, edgeDir, squaredSubRadius);
            auto end = std::chrono::high_resolution_clock::now();
            totalDuration += static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

            totalLight += light;
            ++numSubdivisions;
            edge.data.throughput.value *= light;
            progress.Update();
        }
    });
    progress.Done();

    std::cout << "average light: " << totalLight / static_cast<float>(numSubdivisions) << std::endl;
    std::cout << "average duration: " << totalDuration / numSubdivisions << std::endl;
}

float Subdivider::Subdivide(int vertexId, const Sphere& sphere, Vector3f inDirection, float squaredSearchRadius) {
    GeometricPrimitive primitive(&sphere, nullptr, nullptr, sphereInterface);
    util::PrimitiveData primitiveData(&primitive);
    util::MediumData localMediumData = mediumData;
    localMediumData.primitiveData = primitiveData;

    FreeGraphBuilder graphBuilder(localMediumData, inDirection, sampler, config.graphBuilder, true, vertexId, squaredSearchRadius);
    FreeGraph graph = graphBuilder.TracePaths();
    graphBuilder.ComputeTransmittance(graph);

    FreeLightingCalculator lightingCalculator(graph, localMediumData, inDirection, sampler, config.lightingCalculator, true);
    lightingCalculator.ComputeFinalLight();

    float averageLight = 0;
    for (auto& [id, vertex] : graph.GetVertices())
        averageLight += vertex.data.lightScalar;

    float numVertices = static_cast<float>(graph.GetVertices().size());
    float res = numVertices == 0 ? 1 : averageLight / numVertices;
    return res;
}
}
