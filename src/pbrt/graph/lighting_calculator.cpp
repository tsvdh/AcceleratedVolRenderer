#include "lighting_calculator.h"

namespace graph {

LightingCalculator::LightingCalculator(const UniformGraph& grid, const util::MediumData& mediumData,
                                       const std::vector<RefConst<Vertex>>& litVertices)
    : grid(grid), mediumData(mediumData), litVertices(litVertices) {

    if (!mediumData.medium.Is<HomogeneousMedium>())
        ErrorExit("Only homogeneous media supported");

    if (!HasSequentialIds())
        ErrorExit("Grid must have sequential ids for mapping to matrices");

    numVertices = static_cast<int>(grid.GetVertices().size());
}

UniformGraph LightingCalculator::GetFinalLightGrid(int numIterations) const {
    if (numIterations < 0  || numIterations > 1)
        ErrorExit("Must be zero or one iteration");

    Eigen::VectorXf light = GetLightVector();

    if (numIterations == 1)
        light += GetTransmittanceMatrix() * light;

    UniformGraph finalGrid(grid.GetSpacing());

    for (auto& pair : grid.GetVertices()) {
        auto& v = pair.second;
        finalGrid.AddVertex(v.id, v.coors.value(), VertexData{{}, SampledSpectrum(light[v.id])});
    }

    return finalGrid;
}

bool LightingCalculator::HasSequentialIds() const {
    int total = 0;
    int max = 0;

    for (auto& pair : grid.GetVertices()) {
        total += pair.first;
        max = std::max(max, pair.first);
    }

    int oddInMiddle = max % 2 == 1 ? (max / 2 + 1) : 0;
    int totalExpected = (max + 1) * (max / 2) + oddInMiddle;

    return total == totalExpected;
}

Eigen::VectorXf LightingCalculator::GetLightVector() const {
    Eigen::VectorXf light = Eigen::VectorXf::Zero(numVertices);

    for (auto vertex : litVertices) {
        light(grid.GetVertexConst(vertex.get().coors.value()).value().get().id) = 1;
    }

    return light;
}

Eigen::MatrixXf LightingCalculator::GetPhaseMatrix() const {
    Eigen::MatrixXf phase = Eigen::MatrixXf::Zero(numVertices, numVertices);

    for (int i = 0; i < numVertices; ++i)
        phase(i, i) = 1 / (4 * Pi);

    return phase;
}

Eigen::MatrixXf LightingCalculator::GetGMatrix() const {
    auto& vertices = grid.GetVertices();
    auto& edges = grid.GetEdges();
    Eigen::MatrixXf gMatrix = Eigen::MatrixXf::Zero(numVertices, numVertices);

    for (auto& vertexPair : grid.GetVertices()) {
        for (auto& edgePair : vertexPair.second.outEdges) {
            const Vertex& from = vertexPair.second;
            const Vertex& to = vertices.at(edgePair.first);
            const Edge& edge = edges.at(edgePair.second);

            float T = edge.data.throughput[0];
            float G = 1 / static_cast<float>(std::pow(Length(from.point - to.point), 2));
            gMatrix(to.id, from.id) = T * G;
        }
    }

    return gMatrix;
}

Eigen::MatrixXf LightingCalculator::GetTransmittanceMatrix() const {
    return GetPhaseMatrix() * GetGMatrix() / static_cast<float>(numVertices);
}

}