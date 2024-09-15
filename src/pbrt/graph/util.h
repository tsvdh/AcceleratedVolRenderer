#pragma once

#include <pbrt/cpu/aggregates.h>
#include <pbrt/cpu/primitive.h>
#include <pbrt/util/vecmath.h>

#include <utility>

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
    SampledWavelengths lambda;
    Medium medium;
    BVHAggregate* aggregate;
    Bounds3f bounds;
    Point3f boundsCenter;
    float maxDistToCenter;

    MediumData(const SampledWavelengths& lambda, Primitive accel) : lambda(lambda) {
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

}

namespace graph {

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

}