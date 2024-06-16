#include "vol_boundary.h"

namespace graph {

void VolBoundary::CaptureBoundary(graph::UniformGraph& graph, int horizontalStep, int verticalStep) {
    for (int theta = 0; theta < 360; theta += horizontalStep) {
        float sinTheta = std::sin((float)theta * Pi / 180);
        float cosTheta = std::cos((float)theta * Pi / 180);

        for (int phi = 0; phi < 360; phi += verticalStep) {
            Vector3f dir = SphericalDirection(sinTheta, cosTheta, (float)phi);

        }

    }

}

}