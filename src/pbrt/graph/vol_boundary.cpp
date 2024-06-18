#include "vol_boundary.h"
#include "pbrt/util/progressreporter.h"
#include <pbrt/media.h>
#include <pbrt/cpu/aggregates.h>

namespace graph {

VolBoundary::VolBoundary(pbrt::Primitive& accel, SampledWavelengths lambda) : lambda(lambda) {
    if (!accel.Is<BVHAggregate>())
        throw std::runtime_error("Accelerator primitive must be a 'BVHAggregate' type");

    std::vector<Primitive> aggregate = accel.Cast<BVHAggregate>()->GetPrimitives();
    if (aggregate.size() != 1)
        throw std::runtime_error("Expected exactly one primitive");

    if (!aggregate[0].Is<GeometricPrimitive>())
        throw std::runtime_error("The primitive must be a 'GeometricPrimitive' type");

    auto primitive = aggregate[0].Cast<GeometricPrimitive>();
    medium = primitive->GetMediumInterface().inside;
    if (!medium || !primitive->GetMediumInterface().IsMediumTransition())
        throw std::runtime_error("Expected one medium in empty space");

    Bounds3f mediumBounds = primitive->Bounds();
    boundsCenter = mediumBounds.pMin + mediumBounds.Diagonal() / 2;
    maxDistToCenter = Length(mediumBounds.Diagonal() / 2);
}

UniformGraph* VolBoundary::CaptureBoundary(float graphSpacing, int horizontalStep, int verticalStep) {
    ProgressReporter progress((360 / horizontalStep) * (360 / verticalStep), "Capturing volume boundary", false);

    auto graph = new UniformGraph(graphSpacing);

    int numSteps = 100;
    float stepSize = maxDistToCenter / (float)numSteps;

    for (int theta = 0; theta < 360; theta += horizontalStep) {
        float sinTheta = std::sin((float)theta * Pi / 180);
        float cosTheta = std::cos((float)theta * Pi / 180);

        for (int phi = 0; phi < 360; phi += verticalStep) {
            Vector3f dir = SphericalDirection(sinTheta, cosTheta, (float)phi);
            Point3f origin(boundsCenter - dir * maxDistToCenter);
            graph->AddVertex(origin);

            Vector3f xVector;
            Vector3f yVector;
            CoordinateSystem(dir, &xVector, &yVector);
            xVector *= stepSize;
            yVector *= stepSize;

            ScratchBuffer buffer;
            for (int i = -numSteps; i < numSteps; ++i) {
                for (int j = -numSteps; j < numSteps; ++j) {
                    Point3f newOrigin = origin + xVector * i + yVector * j;
                    Ray gridRay(newOrigin, dir);

                    auto iter = medium.SampleRay(gridRay, Infinity, lambda, buffer);
                    while (true) {
                        pstd::optional<RayMajorantSegment> segment = iter.Next();
                        if (!segment)
                            break;
                        if (segment->sigma_maj[0] != 0) {
                            graph->AddVertex(gridRay(segment->tMin));
                            break;
                        }
                    }
                }
            }
            progress.Update();
        }
    }
    progress.Done();

    return graph;
}

}