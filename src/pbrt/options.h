// pbrt is Copyright(c) 1998-2020 Matt Pharr, Wenzel Jakob, and Greg Humphreys.
// The pbrt source code is licensed under the Apache License, Version 2.0.
// SPDX: Apache-2.0

#ifndef PBRT_OPTIONS_H
#define PBRT_OPTIONS_H

#include <pbrt/pbrt.h>
#include <pbrt/util/log.h>
#include <pbrt/util/pstd.h>
#include <pbrt/util/vecmath.h>

#include <optional>
#include <string>
#include <unordered_set>

namespace pbrt {

// RenderingCoordinateSystem Definition
enum class RenderingCoordinateSystem { Camera, CameraWorld, World };
std::string ToString(const RenderingCoordinateSystem &);

// BasicPBRTOptions Definition
struct BasicPBRTOptions {
    int seed = 0;
    bool quiet = false;
    bool disablePixelJitter = false, disableWavelengthJitter = false;
    bool disableTextureFiltering = false;
    bool disableImageTextures = false;
    bool forceDiffuse = false;
    bool useGPU = false;
    bool wavefront = false;
    bool interactive = false;
    bool fullscreen = false;
    RenderingCoordinateSystem renderingSpace = RenderingCoordinateSystem::CameraWorld;
};

// GraphOptionsDefinition
struct GraphOptions {
    std::string sceneFile;
    bool debug = false;
    pstd::optional<Point2i> samplingResolution;
    pstd::optional<std::string> dataFile;
};

// PBRTOptions Definition
struct PBRTOptions : BasicPBRTOptions {
    pstd::optional<int> maxdepth;
    int nThreads = 0;
    LogLevel logLevel = LogLevel::Error;
    std::string logFile;
    bool logUtilization = false;
    bool writePartialImages = false;
    bool recordPixelStatistics = false;
    bool printStatistics = false;
    pstd::optional<int> pixelSamples;
    pstd::optional<int> gpuDevice;
    bool quickRender = false;
    bool upgrade = false;
    std::string imageFile;
    std::string mseReferenceImage, mseReferenceOutput;
    bool mseFinalOnly = false;
    std::string debugStart;
    std::string displayServer;
    pstd::optional<Bounds2f> cropWindow;
    pstd::optional<Bounds2i> pixelBounds;
    pstd::optional<Point2i> pixelMaterial;
    Float displacementEdgeScale = 1;

    GraphOptions graph;

    std::string ToString() const;
};

// Options Global Variable Declaration
extern PBRTOptions *Options;

#if defined(PBRT_BUILD_GPU_RENDERER)
#if defined(__CUDACC__)
extern __constant__ BasicPBRTOptions OptionsGPU;
#endif  // __CUDACC__

void CopyOptionsToGPU();
#endif  // PBRT_BUILD_GPU_RENDERER

// Options Inline Functions
PBRT_CPU_GPU inline const BasicPBRTOptions &GetOptions();

PBRT_CPU_GPU inline const BasicPBRTOptions &GetOptions() {
#if defined(PBRT_IS_GPU_CODE)
    return OptionsGPU;
#else
    return *Options;
#endif
}

}  // namespace pbrt

#endif  // PBRT_OPTIONS_H
