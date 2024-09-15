#include "lighting_calculator.h"

#include "pbrt/lights.h"

namespace graph {

LightingCalculator::LightingCalculator(const UniformGraph& grid, const util::MediumData& mediumData,
                                       DistantLight* light, const std::vector<RefConst<Vertex>>& litVertices)
    : grid(grid), mediumData(mediumData), light(light), litVertices(litVertices) {

    if (!mediumData.medium.Is<HomogeneousMedium>())
        ErrorExit("Only homogeneous media supported");

    if (!HasSequentialIds())
        ErrorExit("Grid must have sequential ids for mapping to matrices");

    numVertices = static_cast<int>(grid.GetVertices().size());
}

UniformGraph LightingCalculator::GetFinalLightGrid(int numIterations) const {
    if (numIterations < 0  || numIterations > 1)
        ErrorExit("Must be zero or one iteration");

    auto light = GetLightVector();

    for (int i = 0; i < NSpectrumSamples; ++i) {
        if (numIterations == 1)
            light += GetTransmittanceMatrix() * light;
    }

    UniformGraph finalGrid(grid.GetSpacing());

    for (auto& pair : grid.GetVertices()) {
        auto& v = pair.second;
        finalGrid.AddVertex(v.id, v.coors.value(), VertexData{{}, light[v.id]});
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

Matrix<SampledSpectrum, Dynamic, 1> LightingCalculator::GetLightVector() const {
    // lambda only relevant parameter for distant light
    SampledSpectrum L = light->SampleLi(LightSampleContext{Interaction()},
                                        Point2f(), mediumData.lambda, false).value().L;

    Matrix<SampledSpectrum, Dynamic, 1> lightVector = Matrix<SampledSpectrum, Dynamic, 1>::Zero(numVertices);

    for (auto vertex : litVertices) {
        lightVector(grid.GetVertexConst(vertex.get().coors.value()).value().get().id) = L;
    }

    return lightVector;
}

Matrix<SampledSpectrum, Dynamic, Dynamic> LightingCalculator::GetPhaseMatrix() const {
    Matrix<SampledSpectrum, Dynamic, Dynamic> phase = Matrix<SampledSpectrum, Dynamic, Dynamic>::Zero(numVertices, numVertices);

    for (int i = 0; i < numVertices; ++i)
        phase(i, i) = SampledSpectrum(1 / (4 * Pi));

    return phase;
}

Matrix<SampledSpectrum, Dynamic, Dynamic> LightingCalculator::GetGMatrix() const {
    auto& vertices = grid.GetVertices();
    auto& edges = grid.GetEdges();
    Matrix<SampledSpectrum, Dynamic, Dynamic> gMatrix = Matrix<SampledSpectrum, Dynamic, Dynamic>::Zero(numVertices, numVertices);

    for (auto& vertexPair : grid.GetVertices()) {
        for (auto& edgePair : vertexPair.second.outEdges) {
            const Vertex& from = vertexPair.second;
            const Vertex& to = vertices.at(edgePair.first);
            const Edge& edge = edges.at(edgePair.second);

            SampledSpectrum T = edge.data.throughput;
            float G = 1 / static_cast<float>(std::pow(Length(from.point - to.point), 2));
            gMatrix(to.id, from.id) = T * G;
        }
    }

    return gMatrix;
}

Matrix<SampledSpectrum, Dynamic, Dynamic> LightingCalculator::GetTransmittanceMatrix() const {
    return GetPhaseMatrix() * GetGMatrix() / SampledSpectrum(static_cast<float>(numVertices));
}

}