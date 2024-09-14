#pragma once

#include "graph.h"
#include "pbrt/graph/deps/Eigen/Dense"

namespace graph {

class LightingCalculator {
public:
    LightingCalculator(const UniformGraph& grid, const util::MediumData& mediumData,
                       const std::vector<RefConst<Vertex>>& litVertices);
    [[nodiscard]] UniformGraph GetFinalLightGrid(int numIterations) const;

private:
    [[nodiscard]] bool HasSequentialIds() const;
    [[nodiscard]] Eigen::VectorXf GetLightVector() const;
    [[nodiscard]] Eigen::MatrixXf GetTransmittanceMatrix() const;
    [[nodiscard]] Eigen::MatrixXf GetGMatrix() const;
    [[nodiscard]] Eigen::MatrixXf GetPhaseMatrix() const;

    const UniformGraph& grid;
    const util::MediumData& mediumData;
    std::vector<RefConst<Vertex>> litVertices;
    int numVertices;
};

}