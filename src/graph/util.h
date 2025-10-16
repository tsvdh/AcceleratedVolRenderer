#pragma once

#include <pbrt/cpu/aggregates.h>
#include <pbrt/cpu/primitive.h>
#include <pbrt/util/vecmath.h>

#include <utility>

#include "deps/json.hpp"
#include "deps/nanoflann.hpp"
#include "pbrt/media.h"
#include "pbrt/options.h"
#include "pbrt/shapes.h"
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

struct PrimitiveData {
    Primitive primitive;
    Bounds3f bounds;
    Point3f boundsCenter;
    float maxDistToCenter = 0;

    PrimitiveData() = default;

    explicit PrimitiveData(const Primitive& primitive) {
        this->primitive = primitive;
        bounds = primitive.Bounds();
        boundsCenter = bounds.pMin + bounds.Diagonal() / 2;
        maxDistToCenter = Length(bounds.Diagonal() / 2);
    }
};

struct MediumData {
    SampledWavelengths defaultLambda;
    Medium medium;
    PrimitiveData primitiveData;

    MediumData() = default;

    MediumData(Primitive accel, const SampledWavelengths& defaultLambda) : defaultLambda(defaultLambda), primitiveData(accel) {
        if (!accel.Is<BVHAggregate>())
            ErrorExit("Accelerator primitive must be a 'BVHAggregate' type");

        std::vector<Primitive>& primitives = accel.Cast<BVHAggregate>()->GetPrimitives();

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
    explicit VerticesHolder(std::vector<Point3f> list) : verticesList(std::move(list)) {}

    [[nodiscard]] std::vector<Point3f>& GetList() { return verticesList; }
    [[nodiscard]] const std::vector<Point3f>& GetListConst() const { return verticesList; }

    [[nodiscard]] size_t kdtree_get_point_count() const { return verticesList.size(); }

    [[nodiscard]] Float kdtree_get_pt(size_t index, size_t dim) const {
        int _index = static_cast<int>(index);
        switch (dim) {
        case 0:
            return verticesList[_index].x;
        case 1:
            return verticesList[_index].y;
        case 2:
            return verticesList[_index].z;
        default:
            ErrorExit("Impossible dimension");
        }
    }

    // ReSharper disable once CppMemberFunctionMayBeStatic
    template <class BBOX> bool kdtree_get_bbox(BBOX& bb) const { return false; }

private:
    std::vector<Point3f> verticesList; // vertex id, point
};

inline DistantLight* GetLight(std::vector<Light>& lights) {
    if (lights.size() != 1)
        ErrorExit("Expected exactly one light source");

    if (!lights[0].Is<DistantLight>())
        ErrorExit("Expected a directional light");

    return lights[0].Cast<DistantLight>();
}

static std::vector<Point3f> GetSphereSurfacePoints(Point3f center, float radius, float equatorStepDegrees) {
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

// ReSharper disable once CppDFAConstantParameter
static std::vector<Point3f> GetDiskPoints(Point3f center, float radius, int numPointsOnRadius, Vector3f direction) {
    if (numPointsOnRadius == 0)
        return {center};

    Vector3f xVector;
    Vector3f yVector;
    CoordinateSystem(direction, &xVector, &yVector);

    float stepSize = radius / static_cast<float>(numPointsOnRadius + 1);
    xVector *= stepSize;
    yVector *= stepSize;

    std::vector<Point3f> points;
    points.reserve(static_cast<int>(Pi * static_cast<float>(Sqr(numPointsOnRadius))));

    for (int x = -numPointsOnRadius; x <= numPointsOnRadius; ++x) {
        for (int y = -numPointsOnRadius; y <= numPointsOnRadius; ++y) {
            Point3f newPoint = center + x * xVector + y * yVector;
            if (Distance(newPoint, center) <= radius)
                points.push_back(newPoint);
        }
    }

    points.shrink_to_fit();
    return points;
}

inline int GetDiskPointsSize(int numPointOnRadius) {
    return static_cast<int>(GetDiskPoints(Point3f(), 1, numPointOnRadius, Vector3f(1, 0, 0)).size());
}

static std::vector<Point3f> GetSphereVolumePoints(float radius, int numPointsOnRadius) {
    if (numPointsOnRadius == 0)
        return {Point3f()};

    Point3f center;
    float stepSize = radius / static_cast<float>(numPointsOnRadius + 1);

    std::vector<Point3f> points;
    points.reserve(static_cast<uint64_t>(4.f / 3.f * Pi * std::pow(numPointsOnRadius, 3)));

    for (int x = -numPointsOnRadius; x <= numPointsOnRadius; ++x) {
        for (int y = -numPointsOnRadius; y <= numPointsOnRadius; ++y) {
            for (int z = -numPointsOnRadius; z <= numPointsOnRadius; ++z) {
                Point3f newPoint = {
                    stepSize * static_cast<float>(x),
                    stepSize * static_cast<float>(y),
                    stepSize * static_cast<float>(z)
                };
                if (Distance(newPoint, center) <= radius)
                    points.push_back(newPoint);
            }
        }
    }

    points.shrink_to_fit();
    return points;
}

static std::vector<Point3f> GetSphereVolumePointsRandom(float radius, Point3f center, int numPoints, Sampler sampler) {
    std::vector<Point3f> points;
    points.reserve(numPoints);

    while (points.size() < numPoints) {
        Point3f point((sampler.Get1D() - 0.5) * 2 * radius,
                      (sampler.Get1D() - 0.5) * 2 * radius,
                      (sampler.Get1D() - 0.5) * 2 * radius);

        if (Length(Vector3f(point)) < radius)
            points.push_back(point + center);
    }

    return points;
}

inline int GetSphereVolumePointsSize(int numPointsOnRadius) {
    return static_cast<int>(GetSphereVolumePoints(1, numPointsOnRadius).size());
}

class SpherePointsMaker {
public:
    SpherePointsMaker(float radius, int numPointsOnRadius)
    : radius(radius), numPointsOnRadius(numPointsOnRadius), points(GetSphereVolumePoints(radius, numPointsOnRadius)) {}

    std::vector<Point3f> GetSpherePointsFor(Point3f center) {
        std::vector<Point3f> newPoints = points;
        for (Point3f& newPoint : newPoints)
            newPoint += center;
        return newPoints;
    }

private:
    float radius;
    int numPointsOnRadius;
    std::vector<Point3f> points;
};

inline std::vector<Point3f> GetSphereVolumePoints(float radius, int numPointsOnRadius, Point3f center) {
    return SpherePointsMaker(radius, numPointsOnRadius).GetSpherePointsFor(center);
}

inline Sphere MakeSphere(float radius) {
    return Sphere(nullptr, nullptr, false, false,
                  radius, -radius, radius, 360);
}

class SphereMaker {
public:
    explicit SphereMaker(float radius) : radius(radius), sphere(MakeSphere(radius)) {}

    SphereContainer GetSphereFor(Point3f center) {
        return SphereContainer(sphere, Translate(Vector3f(center)));
    }

private:
    float radius;
    Sphere sphere;
};

inline SphereContainer MakeSphere(float radius, Point3f center) {
    return SphereMaker(radius).GetSphereFor(center);
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

inline float Transmittance(const MediumInteraction& p0, Point3f p1, const SampledWavelengths& lambda, Sampler sampler) {
    RNG rng(Hash(sampler.Get1D()), Hash(sampler.Get1D()));

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

inline float SampleTransmittance(const Ray& ray, float tMax, Sampler sampler, const MediumData& mediumData) {
    // Initialize _RNG_ for sampling the majorant transmittance
    RNG rng(Hash(sampler.Get1D()), Hash(sampler.Get1D()));

    float Tr = 1;

    // Sample new point on ray
    SampleT_maj(
        ray, tMax, sampler.Get1D(), rng, mediumData.defaultLambda,
        [&](Point3f p, MediumProperties mp, SampledSpectrum sigma_maj, SampledSpectrum T_maj) {
            SampledSpectrum sigma_n = ClampZero(sigma_maj - mp.sigma_a - mp.sigma_s);

            // ratio-tracking: only evaluate null scattering
            Tr *= sigma_n[0] / sigma_maj[0];

            if (Tr == 0)
                return false;

            return true;
        });

    return Tr;
}

inline bool SampleScatter(const Ray& ray, float tMax, Sampler sampler, const MediumData& mediumData) {
    // Initialize _RNG_ for sampling the majorant transmittance
    RNG rng(Hash(sampler.Get1D()), Hash(sampler.Get1D()));

    bool scattered = false;

    // Sample new point on ray
    SampleT_maj(
        ray, tMax, sampler.Get1D(), rng, mediumData.defaultLambda,
        [&](Point3f p, MediumProperties mp, SampledSpectrum sigma_maj, SampledSpectrum T_maj) {
            // Compute medium event probabilities for interaction
            Float pAbsorb = mp.sigma_a[0] / sigma_maj[0];
            Float pScatter = mp.sigma_s[0] / sigma_maj[0];
            Float pNull = std::max<Float>(0, 1 - pAbsorb - pScatter);

            CHECK_GE(1 - pAbsorb - pScatter, -1e-6);
            // Sample medium scattering event type and update path
            Float um = rng.Uniform<Float>();
            int mode = SampleDiscrete({pAbsorb, pScatter, pNull}, um);

            if (mode == 0) {
                // Handle absorption along ray path
                return false;
            }
            if (mode == 1) {
                // Handle scattering along ray path
                scattered = true;
                return false;
            }
            else {
                return true;
            }
        });

    return scattered;
}

enum HitsType {
    OutsideTwoHits, OutsideOneHit, OutsideZeroHits, InsideOneHit
};

struct HitsResult {
    std::vector<float> tHits;
    std::vector<ShapeIntersection> intersections;
    HitsType type;

    bool RayEntersVolume() {
        return type == InsideOneHit || type == OutsideTwoHits;
    };
};

static HitsResult GetHits(const Primitive& primitive, RayDifferential ray, const MediumData& mediumData) {
    std::vector<float> tHits;
    std::vector<ShapeIntersection> intersections;

    float distBack = mediumData.primitiveData.maxDistToCenter * 2;
    RayDifferential rayOutside = ray;
    rayOutside.o -= ray.d * distBack;

    pstd::optional<ShapeIntersection> firstIntersect = primitive.Intersect(ray, Infinity);
    if (!firstIntersect) {
        return {tHits, intersections, OutsideZeroHits};
    }
    tHits.push_back(firstIntersect->tHit);
    intersections.push_back(firstIntersect.value());
    firstIntersect->intr.SkipIntersection(&ray, Infinity);

    pstd::optional<ShapeIntersection> secondIntersect = primitive.Intersect(ray, Infinity);
    if (secondIntersect) {
        tHits.push_back(tHits[0] + secondIntersect->tHit);
        intersections.push_back(secondIntersect.value());
        return {tHits, intersections, OutsideTwoHits};
    }

    pstd::optional<ShapeIntersection> intersectFromFar = primitive.Intersect(rayOutside, Infinity);
    if (!intersectFromFar) {
        // Warning("Rare case of same ray not hitting primitive");
        return {{}, {}, OutsideZeroHits};
    }

    float adjustedFromFarTHit = intersectFromFar->tHit - distBack;
    float tHitDiff = std::abs(firstIntersect->tHit - adjustedFromFarTHit);

    HitsType type = tHitDiff < std::pow(10, -3) ? OutsideOneHit : InsideOneHit;
    if (type == OutsideOneHit) {
        // Warning("Rare case of outside ray hitting primitive once");
        return {{}, {}, OutsideZeroHits};
    }

    return {tHits, intersections, type};
}

inline HitsResult GetHits(const Sphere& sphere, const RayDifferential& ray, const MediumData& mediumData) {
    GeometricPrimitive primitive(&sphere, nullptr, nullptr, {});
    return GetHits(&primitive, ray, mediumData);
}

inline float GetSameSpotRadius(const MediumData& mediumData) {
    return mediumData.primitiveData.maxDistToCenter * 2 / 1000;
}

struct StartEndT {
    float startT;
    float endT;
    float startScatterT;
    float endScatterT;
    bool sphereRayNotInMedium;

    void SkipForward(float time) {
        startT -= time;
        endT -= time;
        startScatterT -= time;
        endScatterT -= time;
    }
};

inline StartEndT GetStartEndT(const HitsResult& mediumHits, const HitsResult& sphereHits) {
    StartEndT startEnd{};

    std::vector<float> mediumTHits = mediumHits.tHits;
    std::vector<float> sphereTHits = sphereHits.tHits;

    float mediumEntry = mediumHits.type == OutsideTwoHits ? mediumTHits[0] : 0;
    float sphereEntry = sphereHits.type == OutsideTwoHits ? sphereTHits[0] : 0;
    float mediumExit = mediumHits.type == OutsideTwoHits ? mediumTHits[1] : mediumTHits[0];
    float sphereExit = sphereHits.type == OutsideTwoHits ? sphereTHits[1] : sphereTHits[0];

    startEnd.startT = mediumEntry;
    startEnd.endT = std::min(mediumExit, sphereExit);
    startEnd.startScatterT = std::max(mediumEntry, sphereEntry);
    startEnd.endScatterT = startEnd.endT;

    startEnd.sphereRayNotInMedium = startEnd.endT < startEnd.startScatterT || startEnd.endScatterT < startEnd.startT;

    return startEnd;
}

class Averager;

struct SamplesStore {
    float value = -1;
    float numSamples = 0;

    void AddSample(float sampleValue) {
        AddSamples({sampleValue, 1});
    }

    void AddSamples(const SamplesStore& other) {
        this->value *= this->numSamples;
        this->value += other.value * other.numSamples;
        this->numSamples += other.numSamples;
        this->value /= this->numSamples != 0 ? this->numSamples : -1;
    }

    void RemoveSample(float sampleValue) {
        AddSamples({sampleValue, -1});
    }

    void FillWithAverager(Averager& averager);
};

class Averager {
public:
    explicit Averager(int numValues = 0) {
        values.reserve(numValues);
        weights.reserve(numValues);
    }

    void AddValue(float value, float weight = 1) {
        values.push_back(value);
        weights.push_back(weight);
    }

    const std::vector<float>& GetValues() { return values; }

    SamplesStore ToSamplesStore() {
        return {GetAverage(), static_cast<float>(values.size())};
    }

    float GetAverage() {
        return GetAverage(true);
    }

    float GetAverage(bool useWeights) {
        if (values.empty())
            return 0;

        float average = 0;
        float totalWeight = 0;
        for (int i = 0; i < values.size(); ++i) {
            float weight = useWeights ? weights[i] : 1;
            average += values[i] * weight;
            totalWeight += weight;
        }

        if (totalWeight == 0)
            return 0;

        average /= totalWeight;
        return average;
    }

    std::tuple<float, float, float> GetInfo() {
        if (values.empty())
            return {0, 0, 0};

        float average = GetAverage();

        float variance = 0;
        for (float value : values)
            variance += Sqr(value - average);
        variance /= static_cast<float>(values.size());

        float std = std::sqrt(variance);
        return {average, std, variance};
    }

    std::string PrintInfo() {
        auto [avg, std, var] = GetInfo();
        return StringPrintf("average %s, std %s, var %s", avg, std, var);
    }

    float GetDenoisedAverage() {
        if (values.empty())
            return 0;

        auto [average, std, variance] = GetInfo();

        float denoisedAverage = 0;
        float numRemaining = 0;
        for (float value : values) {
            if (std::abs(value - average) < std) {
                denoisedAverage += value;
                ++numRemaining;
            }
        }
        if (numRemaining == 0)
            return average;

        denoisedAverage /= numRemaining;
        return denoisedAverage;
    }

private:
    std::vector<float> values;
    std::vector<float> weights;
};

inline void SamplesStore::FillWithAverager(Averager& averager) {
    this->value = averager.GetAverage();
    this->numSamples = averager.GetValues().size();
}

inline std::string formatTime(int totalSeconds) {
    int hours = totalSeconds / 3600;
    int minutes = totalSeconds / 60 - hours * 60;
    int seconds = totalSeconds - hours * 3600 - minutes * 60;

    if (hours > 0)
        return StringPrintf("%ih %im %is", hours, minutes, seconds);
    if (minutes > 0)
        return StringPrintf("%im %is", minutes, seconds);

    return StringPrintf("%is", seconds);
}

inline std::string formatTime(float totalSeconds) {
    float whole;
    float fractional = std::modf(totalSeconds, &whole);
    std::string formattedTime = formatTime(static_cast<int>(whole));
    return StringPrintf("%s.%ss",
        formattedTime.substr(0, formattedTime.size() - 1),
        StringPrintf("%.1f", fractional).substr(2, 2));
}

class TaskTimeTracker {
public:
    explicit TaskTimeTracker(std::string taskName, bool quiet) : taskName(std::move(taskName)), quiet(quiet) {}

    void Start() {
        startTime = std::chrono::steady_clock::now();
        if (!quiet)
            std::cout << StringPrintf("%s... ", taskName);
    }

    void End() {
        auto endTime = std::chrono::steady_clock::now();
        float elapsedSeconds = std::chrono::duration<float>(endTime - startTime).count();
        if (!quiet)
            std::cout << StringPrintf("done (%s)", formatTime(elapsedSeconds)) << std::endl;
    }

private:
    std::string taskName;
    bool quiet;
    std::chrono::time_point<std::chrono::steady_clock> startTime;
};

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
    grid_lighting,
    outline
};
const std::vector<std::string> descriptionNames{
    "basic",
    "surface",
    "paths",
    "search_queue",
    "search_surface",
    "grid",
    "grid_transmittance",
    "grid_lighting",
    "outline"
};

inline std::string GetDescriptionName(Description desc) {
    return descriptionNames[desc];
}

using nlohmann::json;

struct Config;
struct GraphBuilderConfig;
struct LightingCalculatorConfig;
struct ReinforcementConfig;
struct EdgeReinforcementConfig;
struct NeighbourReinforcementConfig;

struct ReinforcementConfig {
    bool active;
    float unsatisfiedAllowedRatio;
    int reinforcementRays;
};

struct EdgeReinforcementConfig : ReinforcementConfig {
    int edgesForNotSparse;
};

struct NeighbourReinforcementConfig : ReinforcementConfig {
    int neighboursForNotSparse;
    float neighbourRangeModifier;
};

struct GraphBuilderConfig {
    // general
    float radiusModifier;
    int maxDepth;
    int neighboursForRenderSearchRange;

    // path tracing
    int dimensionSteps;
    int iterationsPerStep;

    // sparse reinforcement
    EdgeReinforcementConfig edgeReinforcement;
    NeighbourReinforcementConfig neighbourReinforcement;
};

struct LightingCalculatorConfig {
    int lightIterations;
    int pointsOnRadiusLight;
    std::vector<int> bounces;
    bool runInParallel;
};

struct Config {
    GraphBuilderConfig graphBuilder;
    LightingCalculatorConfig lightingCalculator;
};

inline void from_json(const json& jsonObject, ReinforcementConfig& reinforcementConfig) {
    jsonObject.at("active").get_to(reinforcementConfig.active);
    jsonObject.at("unsatisfiedAllowedRatio").get_to(reinforcementConfig.unsatisfiedAllowedRatio);
    jsonObject.at("reinforcementRays").get_to(reinforcementConfig.reinforcementRays);
}

inline void from_json(const json& jsonObject, EdgeReinforcementConfig& edgeReinforcementConfig) {
    auto edgeReinforcement = jsonObject.at("edgeReinforcement");
    from_json(edgeReinforcement, static_cast<ReinforcementConfig&>(edgeReinforcementConfig));

    edgeReinforcement.at("edgesForNotSparse").get_to(edgeReinforcementConfig.edgesForNotSparse);
}

inline void from_json(const json& jsonObject, NeighbourReinforcementConfig& neighbourReinforcementConfig) {
    auto neighbourReinforcement = jsonObject.at("neighbourReinforcement");
    from_json(neighbourReinforcement, static_cast<ReinforcementConfig&>(neighbourReinforcementConfig));

    neighbourReinforcement.at("neighboursForNotSparse").get_to(neighbourReinforcementConfig.neighboursForNotSparse);
    neighbourReinforcement.at("neighbourRangeModifier").get_to(neighbourReinforcementConfig.neighbourRangeModifier);
}

inline void from_json(const json& jsonObject, GraphBuilderConfig& graphBuilderConfig) {
    auto graphBuilder = jsonObject.at("graphBuilder");
    graphBuilder.at("radiusModifier").get_to(graphBuilderConfig.radiusModifier);
    graphBuilder.at("maxDepth").get_to(graphBuilderConfig.maxDepth);
    graphBuilder.at("neighboursForRenderSearchRange").get_to(graphBuilderConfig.neighboursForRenderSearchRange);

    graphBuilder.at("dimensionSteps").get_to(graphBuilderConfig.dimensionSteps);
    graphBuilder.at("iterationsPerStep").get_to(graphBuilderConfig.iterationsPerStep);

    if (graphBuilder.contains("edgeReinforcement"))
        from_json(graphBuilder, graphBuilderConfig.edgeReinforcement);

    if (graphBuilder.contains("neighbourReinforcement"))
        from_json(graphBuilder, graphBuilderConfig.neighbourReinforcement);
}

inline void from_json(const json& jsonObject, LightingCalculatorConfig& lightingCalculatorConfig) {
    auto lightingCalculator = jsonObject.at("lightingCalculator");
    lightingCalculator.at("lightIterations").get_to(lightingCalculatorConfig.lightIterations);
    lightingCalculator.at("pointsOnRadiusLight").get_to(lightingCalculatorConfig.pointsOnRadiusLight);
    lightingCalculator.at("bounces").get_to(lightingCalculatorConfig.bounces);
    lightingCalculator.at("runInParallel").get_to(lightingCalculatorConfig.runInParallel);
}

inline void from_json(const json& jsonObject, Config& config) {
    from_json(jsonObject, config.graphBuilder);
    from_json(jsonObject, config.lightingCalculator);
}

using namespace pbrt;

inline float ComputeRaysToSphere(const RayDifferential& rayToSphere, const std::optional<RayDifferential>& rayInSphere, const util::StartEndT& startEnd,
                                 const util::MediumData& mediumData, Sampler sampler, int iterations, uint64_t samplingIndex) {
    int yCoor = static_cast<int>(samplingIndex / Options->graph.samplingResolution->x);
    int xCoor = static_cast<int>(samplingIndex - yCoor * Options->graph.samplingResolution->x);

    float distInSphere = startEnd.endScatterT - startEnd.startScatterT;

    util::Averager transmittanceToSphereAverager(iterations);
    util::Averager scatterInSphereAverager(iterations);

    for (int i = 0; i < iterations; ++i) {
        sampler.StartPixelSample({xCoor, yCoor}, i);

        float curDistInSphere = rayInSphere.has_value() ? 0 : distInSphere * sampler.Get1D();
        float curTransmittanceDist = startEnd.startScatterT + curDistInSphere;
        transmittanceToSphereAverager.AddValue(SampleTransmittance(rayToSphere, curTransmittanceDist, sampler, mediumData));
        if (rayInSphere.has_value())
            scatterInSphereAverager.AddValue(SampleScatter(rayInSphere.value(), distInSphere, sampler, mediumData));
    }

    float transmittance = transmittanceToSphereAverager.GetAverage();
    if (rayInSphere.has_value())
        transmittance *= scatterInSphereAverager.GetAverage();

    return transmittance;
}

}