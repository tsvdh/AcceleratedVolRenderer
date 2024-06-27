#pragma once

#include <pbrt/pbrt.h>
#include <pbrt/lights.h>

#include "graph.h"

namespace graph {

using namespace pbrt;

class VolTransmittance {
public:
    VolTransmittance(UniformGraph* boundary, util::MediumData* mediumData)
        : boundary(boundary), mediumData(mediumData) {}

    FreeGraph* CaptureTransmittance(std::vector<Light> lights);

private:
    UniformGraph* boundary;
    util::MediumData* mediumData;
};

}
