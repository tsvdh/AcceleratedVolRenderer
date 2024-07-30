#pragma once

#include <pbrt/pbrt.h>
#include <pbrt/cpu/aggregates.h>
#include <pbrt/cpu/primitive.h>
#include <pbrt/util/vecmath.h>

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

struct MediumData {
    SampledWavelengths lambda;
    Medium medium;
    BVHAggregate* aggregate;
    Bounds3f bounds;
    Point3f boundsCenter;
    float maxDistToCenter;

    MediumData(const SampledWavelengths& lambda, Primitive accel) : lambda(lambda) {
        if (!accel.Is<BVHAggregate>())
            throw std::runtime_error("Accelerator primitive must be a 'BVHAggregate' type");

        aggregate = accel.Cast<BVHAggregate>();
        bounds = aggregate->Bounds();
        boundsCenter = bounds.pMin + bounds.Diagonal() / 2;
        maxDistToCenter = Length(bounds.Diagonal() / 2);

        std::vector<Primitive>& primitives = aggregate->GetPrimitives();

        medium = nullptr;

        for (Primitive& primitive : primitives) {
            if (!primitive.Is<GeometricPrimitive>())
                throw std::runtime_error("The primitive must be a 'GeometricPrimitive' type");

            auto& interface = primitive.Cast<GeometricPrimitive>()->GetMediumInterface();
            if (!interface.IsMediumTransition())
                throw std::runtime_error("The medium must be in empty space");

            if (!interface.inside)
                throw std::runtime_error("There is no medium");

            if (medium && medium != interface.inside)
                throw std::runtime_error("There can be only one medium");

            medium = interface.inside;
        }
    }
};

}