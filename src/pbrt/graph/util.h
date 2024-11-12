#pragma once

#include <pbrt/cpu/aggregates.h>
#include <pbrt/cpu/primitive.h>
#include <pbrt/util/vecmath.h>

#include <utility>

#include "pbrt/media.h"
#include "pbrt/util/error.h"

namespace util {

using namespace pbrt;

struct PointHash {
    std::size_t operator()(Point3i p) const {
        std::string stringP = std::to_string(p.x)
                            + std::to_string(p.y)
                            + std::to_string(p.z);

        return std::hash<std::string>()(stringP);
    }
};

inline std::string FileNameToPath(const std::string& fileName) {
    // convert absolute path backslash to forwardslash
    if (fileName[1] == ':') {
        std::string newPath = fileName;
        std::replace(newPath.begin(), newPath.end(), '\\', '/');
        return newPath;
    }
    // add path pre- and postfix
    return "files/graphs/" + fileName + ".txt";
}

struct MediumData {
    SampledWavelengths defaultLambda;
    Medium medium;
    BVHAggregate* aggregate;
    Bounds3f bounds;
    Point3f boundsCenter;
    float maxDistToCenter;

    explicit MediumData(Primitive accel, const SampledWavelengths& defaultLambda) : defaultLambda(defaultLambda) {
        if (!accel.Is<BVHAggregate>())
            ErrorExit("Accelerator primitive must be a 'BVHAggregate' type");

        aggregate = accel.Cast<BVHAggregate>();
        bounds = aggregate->Bounds();
        boundsCenter = bounds.pMin + bounds.Diagonal() / 2;
        maxDistToCenter = Length(bounds.Diagonal() / 2);

        std::vector<Primitive>& primitives = aggregate->GetPrimitives();

        medium = nullptr;

        for (Primitive& primitive : primitives) {
            if (!primitive.Is<GeometricPrimitive>())
                ErrorExit("The primitive must be a 'GeometricPrimitive' type");

            auto& interface = primitive.Cast<GeometricPrimitive>()->GetMediumInterface();
            if (!interface.IsMediumTransition())
                ErrorExit("The medium must be in empty space");

            if (!interface.inside)
                ErrorExit("There is no medium");

            if (medium && medium != interface.inside)
                ErrorExit("There can be only one medium");

            medium = interface.inside;
        }
    }
};

class VerticesHolder {
public:
    VerticesHolder() = default;
    explicit VerticesHolder(std::vector<std::pair<int, Point3f>> list) : verticesList(std::move(list)) {}

    std::vector<std::pair<int, Point3f>> GetList() { return verticesList; }

    [[nodiscard]] size_t kdtree_get_point_count() const { return verticesList.size(); }

    [[nodiscard]] Float kdtree_get_pt(size_t index, size_t dim) const {
        int _index = static_cast<int>(index);
        switch (dim) {
        case 0:
            return verticesList[_index].second.x;
        case 1:
            return verticesList[_index].second.y;
        case 2:
            return verticesList[_index].second.z;
        default:
            ErrorExit("Impossible dimension");
        }
    }

    // ReSharper disable once CppMemberFunctionMayBeStatic
    template <class BBOX> bool kdtree_get_bbox(BBOX& bb) const { return false; }

private:
    std::vector<std::pair<int, Point3f>> verticesList;
};

inline DistantLight* GetLight(std::vector<Light>& lights) {
    if (lights.size() != 1)
        ErrorExit("Expected exactly one light source");

    if (!lights[0].Is<DistantLight>())
        ErrorExit("Expected a directional light");

    return lights[0].Cast<DistantLight>();
}

inline std::vector<Point3f> GetSpherePoints(Point3f center, float radius, float equatorStepDegrees) {
    if (equatorStepDegrees <= 0 || equatorStepDegrees > 90)
        ErrorExit("Step must be between 0 and 90 degrees");

    Transform toWorld = RotateX(-90);
    std::vector<Point3f> points;

    float curElevation = 0;
    while (curElevation <= 90) {
        float theta = 90 - curElevation;
        float thetaRad = theta * Pi / 180;
        float sinTheta = std::sin(thetaRad);
        float cosTheta = std::cos(thetaRad);

        float thetaDown = 90 + curElevation;
        float thetaRadDown = thetaDown * Pi / 180;
        float sinThetaDown = std::sin(thetaRadDown);
        float cosThetaDown = std::cos(thetaRadDown);

        Vector3f dir = toWorld(SphericalDirection(sinTheta, cosTheta, 0));
        dir.y = 0;
        float curRatio = Length(dir);
        float stepSize = equatorStepDegrees / curRatio;

        // at least two points on circle, except when adding top point
        if (stepSize >= 180 && curRatio > std::pow(10, -6))
            break;

        float phi = 0;
        while (phi < 360) {
            float phiRad = phi * Pi / 180;
            points.push_back(center + toWorld(SphericalDirection(sinTheta, cosTheta, phiRad)) * radius);
            if (curElevation != 0)
                points.push_back(center + toWorld(SphericalDirection(sinThetaDown, cosThetaDown, phiRad)) * radius);

            phi += stepSize;
        }

        curElevation += equatorStepDegrees;
    }

    return points;
}

inline std::pair<Point3i, Point3f> FitToGraph(Point3f p, float spacing) {
    Point3i coors;
    for (int i = 0; i < 3; ++i) {
        coors[i] = static_cast<int>(std::round(p[i] / spacing));
    }

    Point3f fittedPoint = coors * spacing;

    return {coors, fittedPoint};
}

inline Bounds3i FitBounds(const Bounds3f& bounds, float spacing) {
    using std::get;
    return {get<0>(FitToGraph(bounds.pMin, spacing)) - Vector3i(1, 1, 1),
              get<0>(FitToGraph(bounds.pMax, spacing)) + Vector3i(1, 1, 1)};
}

}

namespace graph {

using namespace pbrt;

enum Description {
    basic,
    surface,
    paths,
    search_queue,
    search_surface,
    grid,
    grid_transmittance,
    grid_lighting
};
const std::vector<std::string> descriptionNames{
    "basic",
    "surface",
    "paths",
    "search_queue",
    "search_surface",
    "grid",
    "grid_transmittance",
    "grid_lighting"
};

inline std::string GetDescriptionName(Description desc) {
    return descriptionNames[desc];
}

inline float Transmittance(const MediumInteraction& p0, Point3f p1, const SampledWavelengths& lambda) {
    RNG rng(Hash(p0.p()), Hash(p1));

    Ray ray = p0.SpawnRayTo(p1);
    float Tr = 1;
    if (LengthSquared(ray.d) == 0)
        return Tr;

    SampleT_maj(ray, 1.f, rng.Uniform<Float>(), rng, lambda,
        [&](Point3f p, const MediumProperties& mp, SampledSpectrum sigma_maj, SampledSpectrum T_maj) {

            SampledSpectrum sigma_n = ClampZero(sigma_maj - mp.sigma_a - mp.sigma_s);

            // ratio-tracking: only evaluate null scattering
            Tr *= sigma_n[0] / sigma_maj[0];

            if (Tr == 0)
                return false;

            return true;
        });

    return Tr;
}

inline float Transmittance(const MediumInteraction& p0, const MediumInteraction& p1, const SampledWavelengths& lambda) {
    return Transmittance(p0, p1.p(), lambda);
}

}