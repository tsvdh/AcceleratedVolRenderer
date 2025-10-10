#include "voxel_boundary.h"

#include <pbrt/media.h>

#include <iostream>
#include <queue>

#include "pbrt/shapes.h"
#include "pbrt/util/progressreporter.h"

namespace graph {

FreeGraph VoxelBoundary::CaptureBoundary(float equatorStepSize) const {
    const util::PrimitiveData& primitiveData = mediumData.primitiveData;
    std::vector<Point3f> spherePoints = util::GetSphereSurfacePoints(primitiveData.boundsCenter, primitiveData.maxDistToCenter * 2, equatorStepSize);

    ProgressReporter captureProgress(static_cast<int>(spherePoints.size()), "Capturing volume boundary", false);

    FreeGraph graph;

    int numSteps = 100;
    float stepSize = primitiveData.maxDistToCenter / static_cast<float>(numSteps);

    for (Point3f origin : spherePoints) {
        Vector3f dir = primitiveData.boundsCenter - origin;

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

                auto shapeIsect = primitiveData.primitive.Intersect(gridRay, Infinity);
                if (!shapeIsect)
                    continue;

                shapeIsect->intr.SkipIntersection(&gridRay, shapeIsect->tHit);

                auto iter = mediumData.medium.SampleRay((Ray&)gridRay, Infinity, mediumData.defaultLambda, buffer);
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
    captureProgress.Done();

    return graph;
}

UniformGraph VoxelBoundary::CaptureBoundary(int wantedVertices, float equatorStepSize) {
    FreeGraph graph = CaptureBoundary(equatorStepSize);

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

UniformGraph VoxelBoundary::CaptureBoundary(float spacing, float equatorStepSize) {
    FreeGraph graph = CaptureBoundary(equatorStepSize);

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

void VoxelBoundary::ToSingleLayerAndSaveCast(UniformGraph& boundary) {
    Bounds3i coorBounds = util::FitBounds(mediumData.primitiveData.bounds, boundary.GetSpacing());

    int numVisited = 0;
    auto start = std::chrono::system_clock::now();
    std::cout << "starting to single layer... ";

    std::unordered_set<int> singleLayerSet;

    std::unordered_set<Point3i, util::PointHash>& cast = castCache[boundary.GetGraphId()];

    std::unordered_set<Point3i, util::PointHash> setA;
    std::unordered_set<Point3i, util::PointHash> setB;
    std::unordered_set<Point3i, util::PointHash> setC;

    std::unordered_set<Point3i, util::PointHash>* prevSet = &setA;
    std::unordered_set<Point3i, util::PointHash>* curSet = &setB;
    std::unordered_set<Point3i, util::PointHash>* nextSet = &setC;
    curSet->insert(coorBounds.pMin);

    while (true) {
        for (const Point3i& curPoint : *curSet) {
            ++numVisited;

            for (Point3i neighbour : GetNeighbours(curPoint, coorBounds)) {
                if (auto optVertex = boundary.GetVertex(neighbour); optVertex) {
                    singleLayerSet.insert(optVertex.value().get().id);
                    cast.insert(curPoint);
                }
                else if (prevSet->find(neighbour) == prevSet->end()
                        && curSet->find(neighbour) == curSet->end()) {
                    nextSet->insert(neighbour);
                }
            }
        }

        if (nextSet->empty())
            break;

        prevSet->clear();

        std::unordered_set<Point3i, util::PointHash>* tempSet;
        tempSet = prevSet;
        prevSet = curSet;
        curSet = nextSet;
        nextSet = tempSet;
    }

    std::unordered_set<int> boundaryIds;
    for (auto& pair : boundary.GetVerticesConst())
        boundaryIds.insert(pair.first);

    for (int boundaryId : boundaryIds) {
        if (singleLayerSet.find(boundaryId) == singleLayerSet.end())
            boundary.RemoveVertex(boundaryId);
    }

    auto end = std::chrono::system_clock::now();
    std::cout << "done (" << numVisited << " vertices in " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << "s)" << std::endl;
}

UniformGraph VoxelBoundary::FillInside(UniformGraph& boundary) {
    auto result = castCache.find(boundary.GetGraphId());
    if (result == castCache.end())
        ToSingleLayerAndSaveCast(boundary);

    Bounds3i coorBounds = util::FitBounds(mediumData.primitiveData.bounds, boundary.GetSpacing());

    auto start = std::chrono::system_clock::now();
    std::cout << "starting filling... ";

    std::unordered_set<Point3i, util::PointHash> cast = castCache[boundary.GetGraphId()];

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

        for (Point3i neighbour : GetNeighbours(curPoint, coorBounds)) {
            if (cast.find(neighbour) == cast.end()
                    && visited.find(neighbour) == visited.end() && queueSet.find(neighbour) == queueSet.end()) {
                queue.push(neighbour);
                queueSet.insert(neighbour);
            }
        }
    }

    auto end = std::chrono::system_clock::now();
    std::cout << "done (" << visited.size() << " vertices in " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << "s)" << std::endl;

    return filled;
}

}
