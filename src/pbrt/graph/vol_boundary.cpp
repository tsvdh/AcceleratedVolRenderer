#include "vol_boundary.h"

#include <pbrt/media.h>
#include <pbrt/cpu/aggregates.h>

#include <iostream>
#include <queue>

#include "pbrt/shapes.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

FreeGraph VolBoundary::CaptureBoundary(int horizontalStep, int verticalStep) const {
    ProgressReporter captureProgress((360 / horizontalStep) * (360 / verticalStep), "Capturing volume boundary", false);

    FreeGraph graph;

    int numSteps = 100;
    float stepSize = mediumData->maxDistToCenter / static_cast<float>(numSteps);

    for (int theta = 0; theta < 360; theta += horizontalStep) {
        float sinTheta = std::sin(static_cast<float>(theta) * Pi / 180);
        float cosTheta = std::cos(static_cast<float>(theta) * Pi / 180);

        for (int phi = 0; phi < 360; phi += verticalStep) {
            Vector3f dir = SphericalDirection(sinTheta, cosTheta, static_cast<float>(phi));
            Point3f origin(mediumData->boundsCenter - dir * mediumData->maxDistToCenter * 2);
            // graph->AddVertex(origin);

            Vector3f xVector;
            Vector3f yVector;
            CoordinateSystem(dir, &xVector, &yVector);
            xVector *= stepSize;
            yVector *= stepSize;

            ScratchBuffer buffer;
            for (int i = -numSteps; i < numSteps; ++i) {
                for (int j = -numSteps; j < numSteps; ++j) {
                    Point3f newOrigin = origin + xVector * i + yVector * j;
                    RayDifferential gridRay(newOrigin, dir);

                    auto shapeInter = mediumData->aggregate->Intersect(gridRay, Infinity);
                    if (!shapeInter)
                        continue;

                    shapeInter->intr.SkipIntersection(&gridRay, shapeInter->tHit);

                    auto iter = mediumData->medium.SampleRay((Ray&)gridRay, Infinity, mediumData->lambda, buffer);
                    while (true) {
                        pstd::optional<RayMajorantSegment> segment = iter.Next();
                        if (!segment)
                            break;
                        if (segment->sigma_maj[0] != 0) {
                            graph.AddVertex(gridRay(segment->tMin), new VertexData);
                            break;
                        }
                    }
                }
            }
            captureProgress.Update();
        }
    }
    captureProgress.Done();

    return graph;
}

UniformGraph* VolBoundary::CaptureBoundary(int wantedVertices, int horizontalStep, int verticalStep) const {
    FreeGraph graph = CaptureBoundary(horizontalStep, verticalStep);

    float multRange = 1000;
    int stepsNeeded = static_cast<int>(std::ceil(std::log2(multRange))) + 1;

    ProgressReporter shrinkingProgress(stepsNeeded, "Shrinking boundary graph", false);

    bool spacingGTE1 = graph.ToUniform(1)->GetVertices().size() >= wantedVertices;
    shrinkingProgress.Update();

    float min = 1;
    float max = multRange;
    UniformGraph* curGraph = nullptr;

    for (int i = 0; i < stepsNeeded - 1; ++i) {
        float middle = min + (max - min) / 2;

        curGraph = graph.ToUniform(middle / (spacingGTE1 ? 1 : multRange));
        // std::cout << curGraph->GetSpacing() << std::endl;

        if (curGraph->GetVertices().size() > wantedVertices)
            min = middle;
        else
            max = middle;

        shrinkingProgress.Update();
    }
    shrinkingProgress.Done();

    ToSingleLayer(curGraph);
    return curGraph;
}

UniformGraph* VolBoundary::CaptureBoundary(float spacing, int horizontalStep, int verticalStep) const {
    FreeGraph graph = CaptureBoundary(horizontalStep, verticalStep);

    UniformGraph* spacedGraph = graph.ToUniform(spacing);
    ToSingleLayer(spacedGraph);

    return spacedGraph;
}


inline std::vector<Point3i> GetNeighbours(Point3i p, const Bounds3i& bounds) {
    std::vector<Point3i> neighbours;

    for (int i = 0; i < 6; ++i) {
        Point3i newP(p);
        int index = i / 2;
        int toAdd = i % 2 == 0 ? -1 : 1;
        newP[index] += toAdd;

        if (Inside(newP, bounds)) {
            neighbours.push_back(newP);
        }
    }
    return neighbours;
}

void VolBoundary::ToSingleLayer(UniformGraph* boundary) const {
    using std::get;
    Bounds3i coorBounds(get<0>(boundary->FitToGraph(mediumData->bounds.pMin)) - Vector3i(1, 1, 1),
                        get<0>(boundary->FitToGraph(mediumData->bounds.pMax)) + Vector3i(1, 1, 1));

    UniformGraph search(boundary->GetSpacing());
    UniformGraph layer(boundary->GetSpacing());
    layer.AddVertex(coorBounds.pMin, new VertexData);
    layer.AddVertex(coorBounds.pMax, new VertexData);

    std::unordered_set<int> singleLayerSet;
    std::unordered_set<Point3i, util::PointHash> visited;
    std::queue<Point3i> queue;
    std::unordered_set<Point3i, util::PointHash> queueSet;
    queue.push(coorBounds.pMin);
    queueSet.insert(coorBounds.pMin);


    while (!queue.empty()) {

        Point3i curPoint = queue.front();
        queue.pop();
        queueSet.erase(curPoint);

        for (Point3i neighbour : GetNeighbours(curPoint, coorBounds)) {
            if (auto optVertex = boundary->GetVertex(neighbour); optVertex) {
                singleLayerSet.insert(optVertex.value()->id);
            }
            else if (visited.find(neighbour) == visited.end() && queueSet.find(neighbour) == queueSet.end()) {
                queue.push(neighbour);
                queueSet.insert(neighbour);
                visited.insert(neighbour);
            }
        }
    }

    for (auto [id, vertex] : boundary->GetVertices()) {
        if (singleLayerSet.find(id) == singleLayerSet.end())
            boundary->RemoveVertex(id);
    }
}

}
