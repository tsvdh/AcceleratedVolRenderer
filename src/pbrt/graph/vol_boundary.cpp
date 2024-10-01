#include "vol_boundary.h"

#include <pbrt/media.h>
#include <pbrt/cpu/aggregates.h>

#include <queue>

#include "pbrt/shapes.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

FreeGraph VolBoundary::CaptureBoundary(int horizontalStep, int verticalStep) const {
    ProgressReporter captureProgress((360 / horizontalStep) * (360 / verticalStep), "Capturing volume boundary", false);

    FreeGraph graph;

    int numSteps = 100;
    float stepSize = mediumData.maxDistToCenter / static_cast<float>(numSteps);

    for (int theta = 0; theta < 360; theta += horizontalStep) {
        float sinTheta = std::sin(static_cast<float>(theta) * Pi / 180);
        float cosTheta = std::cos(static_cast<float>(theta) * Pi / 180);

        for (int phi = 0; phi < 360; phi += verticalStep) {
            Vector3f dir = SphericalDirection(sinTheta, cosTheta, static_cast<float>(phi));
            Point3f origin(mediumData.boundsCenter - dir * mediumData.maxDistToCenter * 2);

            Vector3f xVector;
            Vector3f yVector;
            CoordinateSystem(dir, &xVector, &yVector);
            xVector *= stepSize;
            yVector *= stepSize;

            ScratchBuffer buffer;
            for (int i = -numSteps; i <= numSteps; ++i) {
                for (int j = -numSteps; j <= numSteps; ++j) {
                    Point3f newOrigin = origin + xVector * i + yVector * j;
                    RayDifferential gridRay(newOrigin, dir);

                    auto shapeIsect = mediumData.aggregate->Intersect(gridRay, Infinity);
                    if (!shapeIsect)
                        continue;

                    shapeIsect->intr.SkipIntersection(&gridRay, shapeIsect->tHit);

                    auto iter = mediumData.medium.SampleRay((Ray&)gridRay, Infinity, mediumData.lambda, buffer);
                    while (true) {
                        pstd::optional<RayMajorantSegment> segment = iter.Next();
                        if (!segment)
                            break;
                        if (segment->sigma_maj[0] != 0) {
                            graph.AddVertex(gridRay(segment->tMin), VertexData{});
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

UniformGraph VolBoundary::CaptureBoundary(int wantedVertices, int horizontalStep, int verticalStep) {
    FreeGraph graph = CaptureBoundary(horizontalStep, verticalStep);

    float multRange = 1000;
    int stepsNeeded = static_cast<int>(std::ceil(std::log2(multRange))) + 1;

    ProgressReporter shrinkingProgress(stepsNeeded, "Shrinking boundary graph", false);

    bool spacingGTE1 = graph.ToUniform(1).GetVerticesConst().size() >= wantedVertices;
    shrinkingProgress.Update();

    float min = 1;
    float max = multRange;
    UniformGraph curGraph;

    for (int i = 0; i < stepsNeeded - 1; ++i) {
        float middle = min + (max - min) / 2;

        curGraph = graph.ToUniform(middle / (spacingGTE1 ? 1 : multRange));

        if (curGraph.GetVerticesConst().size() > wantedVertices)
            min = middle;
        else
            max = middle;

        shrinkingProgress.Update();
    }
    shrinkingProgress.Done();

    ToSingleLayerAndSaveCast(curGraph);
    return curGraph;
}

UniformGraph VolBoundary::CaptureBoundary(float spacing, int horizontalStep, int verticalStep) {
    FreeGraph graph = CaptureBoundary(horizontalStep, verticalStep);

    UniformGraph spacedGraph = graph.ToUniform(spacing);
    ToSingleLayerAndSaveCast(spacedGraph);

    return spacedGraph;
}

inline std::vector<Point3i> GetNeighbours(Point3i coors, const std::optional<Bounds3i>& bounds) {
    std::vector<Point3i> neighbours;

    for (int i = 0; i < 6; ++i) {
        Point3i newP(coors);
        int index = i / 2;
        int toAdd = i % 2 == 0 ? -1 : 1;
        newP[index] += toAdd;

        if (!bounds.has_value() ? true : Inside(newP, bounds.value())) {
            neighbours.push_back(newP);
        }
    }
    return neighbours;
}

void VolBoundary::ToSingleLayerAndSaveCast(UniformGraph& boundary) {
    using std::get;
    Bounds3i coorBounds(get<0>(boundary.FitToGraph(mediumData.bounds.pMin)) - Vector3i(1, 1, 1),
                        get<0>(boundary.FitToGraph(mediumData.bounds.pMax)) + Vector3i(1, 1, 1));

    std::unordered_set<int> singleLayerSet;
    std::unordered_set<Point3i, util::PointHash>& visited = castCache[&boundary];
    std::queue<Point3i> queue;
    std::unordered_set<Point3i, util::PointHash> queueSet;
    queue.push(coorBounds.pMin);
    queueSet.insert(coorBounds.pMin);

    while (!queue.empty()) {
        Point3i curPoint = queue.front();
        queue.pop();
        queueSet.erase(curPoint);
        visited.insert(curPoint);

        for (Point3i neighbour : GetNeighbours(curPoint, coorBounds)) {
            if (auto optVertex = boundary.GetVertex(neighbour); optVertex) {
                singleLayerSet.insert(optVertex.value().get().id);
            }
            else if (visited.find(neighbour) == visited.end() && queueSet.find(neighbour) == queueSet.end()) {
                queue.push(neighbour);
                queueSet.insert(neighbour);
            }
        }
    }

    for (auto& [id, vertex] : boundary.GetVerticesConst()) {
        if (singleLayerSet.find(id) == singleLayerSet.end())
            boundary.RemoveVertex(id);
    }
}

UniformGraph VolBoundary::FillInside(UniformGraph& boundary) {
    auto result = castCache.find(&boundary);
    if (result == castCache.end())
        ToSingleLayerAndSaveCast(boundary);

    std::unordered_set<Point3i, util::PointHash> cast = castCache[&boundary];

    UniformGraph filled(boundary.GetSpacing());
    std::unordered_set<Point3i, util::PointHash> visited;
    std::queue<Point3i> queue;
    std::unordered_set<Point3i, util::PointHash> queueSet;

    Point3i startPoint = boundary.GetVerticesConst().begin()->second.coors.value();
    queue.push(startPoint);
    queueSet.insert(startPoint);

    while (!queue.empty()) {
        Point3i curPoint = queue.front();
        queue.pop();
        queueSet.erase(curPoint);

        visited.insert(curPoint);
        filled.AddVertex(curPoint * boundary.GetSpacing(), VertexData{});

        for (Point3i neighbour : GetNeighbours(curPoint, {})) {
            if (cast.find(neighbour) == cast.end()
                    && visited.find(neighbour) == visited.end() && queueSet.find(neighbour) == queueSet.end()) {
                queue.push(neighbour);
                queueSet.insert(neighbour);
            }
        }
    }

    return filled;
}

}
