#include "integrator.h"

#include <pbrt/materials.h>
#include <iostream>

namespace graph {

using namespace pbrt;

STAT_COUNTER("Integrator/Volume interactions", volumeInteractions)
STAT_COUNTER("Integrator/Surface interactions", surfaceInteractions)
STAT_PERCENT("Integrator/Regularized BSDFs", regularizedBSDFs, totalBSDFs)

FreeGraph pathGraph;

void GraphVolPathIntegrator::WorkFinished() {
    std::ofstream file("files/graphs/one_pixel.txt");
    file << pathGraph;
    file.close();
}

// GraphVolPathIntegrator Method Definitions
SampledSpectrum GraphVolPathIntegrator::Li(RayDifferential ray, SampledWavelengths& lambda,
                                           Sampler sampler, ScratchBuffer& scratchBuffer,
                                           VisibleSurface* visibleSurf) const {
    // Declare state variables for volumetric path sampling
    SampledSpectrum L(0.f), beta(1.f), r_u(1.f), r_l(1.f);
    bool specularBounce = false, anyNonSpecularBounces = false;
    int depth = 0;
    Float etaScale = 1;
    LightSampleContext prevIntrContext;

    // Init graph path variables
    Path* path = pathGraph.AddPath();
    Vertex* curVertex = nullptr;

    while (true) {
        // Sample segment of volumetric scattering path
        PBRT_DBG("%s\n", StringPrintf("Path tracer depth %d, current L = %s, beta = %s\n",
                                      depth, L, beta)
                .c_str());
        pstd::optional<ShapeIntersection> si = Intersect(ray);

        if (ray.medium) {
            // Sample the participating medium
            bool scattered = false, terminated = false;
            Float tMax = si ? si->tHit : Infinity;
            // Initialize _RNG_ for sampling the majorant transmittance
            uint64_t hash0 = Hash(sampler.Get1D());
            uint64_t hash1 = Hash(sampler.Get1D());
            RNG rng(hash0, hash1);

            SampledSpectrum T_maj = SampleT_maj(
                    ray, tMax, sampler.Get1D(), rng, lambda, // NOLINT(*-slicing)
                    [&](Point3f p, MediumProperties mp, SampledSpectrum sigma_maj,
                        SampledSpectrum T_maj) {
                        // Handle medium scattering event for ray
                        if (!beta) {
                            terminated = true;
                            return false;
                        }
                        ++volumeInteractions;
                        // Add emission from medium scattering event
                        if (depth < maxDepth && mp.Le) {
                            // Compute $\beta'$ at new path vertex
                            Float pdf = sigma_maj[0] * T_maj[0];
                            SampledSpectrum betap = beta * T_maj / pdf;

                            // Compute rescaled path probability for absorption at path vertex
                            SampledSpectrum r_e = r_u * sigma_maj * T_maj / pdf;

                            // Update _L_ for medium emission
                            if (r_e)
                                L += betap * mp.sigma_a * mp.Le / r_e.Average();
                        }

                        // Compute medium event probabilities for interaction
                        Float pAbsorb = mp.sigma_a[0] / sigma_maj[0];
                        Float pScatter = mp.sigma_s[0] / sigma_maj[0];
                        Float pNull = std::max<Float>(0, 1 - pAbsorb - pScatter);

                        CHECK_GE(1 - pAbsorb - pScatter, -1e-6);
                        // Sample medium scattering event type and update path
                        Float um = rng.Uniform<Float>();
                        int mode = SampleDiscrete({pAbsorb, pScatter, pNull}, um);

                        // add edge to graph
                        if (mode == 0 || mode == 1) {
                            auto newVertex = pathGraph.AddVertex(p);
                            if (curVertex) {
                                auto edge = pathGraph.AddEdge(curVertex, newVertex, nullptr, false);
                                path->edges.push_back(edge.value());
                            }
                            curVertex = newVertex;
                        }

                        if (mode == 0) {
                            // Handle absorption along ray path
                            terminated = true;
                            return false;

                        } else if (mode == 1) {
                            // Handle scattering along ray path
                            // Stop path sampling if maximum depth has been reached
                            if (depth++ >= maxDepth) {
                                terminated = true;
                                return false;
                            }

                            // Update _beta_ and _r_u_ for real-scattering event
                            Float pdf = T_maj[0] * mp.sigma_s[0];
                            beta *= T_maj * mp.sigma_s / pdf;
                            r_u *= T_maj * mp.sigma_s / pdf;

                            if (beta && r_u) {
                                // Sample direct lighting at volume-scattering event
                                MediumInteraction intr(p, -ray.d, ray.time, ray.medium,
                                                       mp.phase);

                                L += SampleLd(intr, nullptr, lambda, sampler, beta, r_u);

                                // Sample new direction at real-scattering event
                                Point2f u = sampler.Get2D();
                                pstd::optional<PhaseFunctionSample> ps =
                                        intr.phase.Sample_p(-ray.d, u);
                                if (!ps || ps->pdf == 0)
                                    terminated = true;
                                else {
                                    // Update ray path state for indirect volume scattering
                                    beta *= ps->p / ps->pdf;
                                    r_l = r_u / ps->pdf;
                                    prevIntrContext = LightSampleContext(intr);
                                    scattered = true;
                                    ray.o = p;
                                    ray.d = ps->wi;
                                    specularBounce = false;
                                    anyNonSpecularBounces = true;
                                }
                            }
                            return false;

                        } else {
                            // Handle null scattering along ray path
                            SampledSpectrum sigma_n =
                                    ClampZero(sigma_maj - mp.sigma_a - mp.sigma_s);
                            Float pdf = T_maj[0] * sigma_n[0];
                            beta *= T_maj * sigma_n / pdf;
                            if (pdf == 0)
                                beta = SampledSpectrum(0.f);
                            r_u *= T_maj * sigma_n / pdf;
                            r_l *= T_maj * sigma_maj / pdf;
                            return beta && r_u;
                        }
                    });
            // Handle terminated, scattered, and unscattered medium rays
            if (terminated || !beta || !r_u)
                return L;
            if (scattered)
                continue;

            beta *= T_maj / T_maj[0];
            r_u *= T_maj / T_maj[0];
            r_l *= T_maj / T_maj[0];
        }

        // Handle surviving unscattered rays
        // Add emitted light at volume path vertex or from the environment
        if (!si) {
            // Accumulate contributions from infinite light sources
            for (const auto& light: infiniteLights) {
                if (SampledSpectrum Le = light.Le(ray, lambda); Le) {
                    if (depth == 0 || specularBounce)
                        L += beta * Le / r_u.Average();
                    else {
                        // Add infinite light contribution using both PDFs with MIS
                        Float p_l = lightSampler.PMF(prevIntrContext, light) *
                                    light.PDF_Li(prevIntrContext, ray.d, true);
                        r_l *= p_l;
                        L += beta * Le / (r_u + r_l).Average();
                    }
                }
            }
            break;
        }

        SurfaceInteraction& isect = si->intr;
        if (SampledSpectrum Le = isect.Le(-ray.d, lambda); Le) {
            // Add contribution of emission from intersected surface
            if (depth == 0 || specularBounce)
                L += beta * Le / r_u.Average();
            else {
                // Add surface light contribution using both PDFs with MIS
                Light areaLight(isect.areaLight);
                Float p_l = lightSampler.PMF(prevIntrContext, areaLight) *
                            areaLight.PDF_Li(prevIntrContext, ray.d, true);
                r_l *= p_l;
                L += beta * Le / (r_u + r_l).Average();
            }
        }

        // Get BSDF and skip over medium boundaries
        BSDF bsdf = isect.GetBSDF(ray, lambda, camera, scratchBuffer, sampler);
        if (!bsdf) {
            isect.SkipIntersection(&ray, si->tHit);
            continue;
        }

        // Initialize _visibleSurf_ at first intersection
        if (depth == 0 && visibleSurf) {
            // Estimate BSDF's albedo
            // Define sample arrays _ucRho_ and _uRho_ for reflectance estimate
            constexpr int nRhoSamples = 16;
            const Float ucRho[nRhoSamples] = {
                    0.75741637, 0.37870818, 0.7083487, 0.18935409, 0.9149363, 0.35417435,
                    0.5990858, 0.09467703, 0.8578725, 0.45746812, 0.686759, 0.17708716,
                    0.9674518, 0.2995429, 0.5083201, 0.047338516};
            const Point2f uRho[nRhoSamples] = {
                    Point2f(0.855985, 0.570367), Point2f(0.381823, 0.851844),
                    Point2f(0.285328, 0.764262), Point2f(0.733380, 0.114073),
                    Point2f(0.542663, 0.344465), Point2f(0.127274, 0.414848),
                    Point2f(0.964700, 0.947162), Point2f(0.594089, 0.643463),
                    Point2f(0.095109, 0.170369), Point2f(0.825444, 0.263359),
                    Point2f(0.429467, 0.454469), Point2f(0.244460, 0.816459),
                    Point2f(0.756135, 0.731258), Point2f(0.516165, 0.152852),
                    Point2f(0.180888, 0.214174), Point2f(0.898579, 0.503897)};

            SampledSpectrum albedo = bsdf.rho(isect.wo, ucRho, uRho);

            *visibleSurf = VisibleSurface(isect, albedo, lambda);
        }

        // Terminate path if maximum depth reached
        if (depth++ >= maxDepth)
            return L;

        ++surfaceInteractions;
        // Possibly regularize the BSDF
        if (regularize && anyNonSpecularBounces) {
            ++regularizedBSDFs;
            bsdf.Regularize();
        }

        // Sample illumination from lights to find attenuated path contribution
        if (IsNonSpecular(bsdf.Flags())) {
            L += SampleLd(isect, &bsdf, lambda, sampler, beta, r_u);
            DCHECK(IsInf(L.y(lambda)) == false);
        }
        prevIntrContext = LightSampleContext(isect);

        // Sample BSDF to get new volumetric path direction
        Vector3f wo = isect.wo;
        Float u = sampler.Get1D();
        pstd::optional<BSDFSample> bs = bsdf.Sample_f(wo, u, sampler.Get2D());
        if (!bs)
            break;
        // Update _beta_ and rescaled path probabilities for BSDF scattering
        beta *= bs->f * AbsDot(bs->wi, isect.shading.n) / bs->pdf;
        if (bs->pdfIsProportional)
            r_l = r_u / bsdf.PDF(wo, bs->wi);
        else
            r_l = r_u / bs->pdf;

        PBRT_DBG("%s\n", StringPrintf("Sampled BSDF, f = %s, pdf = %f -> beta = %s",
                                      bs->f, bs->pdf, beta)
                .c_str());
        DCHECK(IsInf(beta.y(lambda)) == false);
        // Update volumetric integrator path state after surface scattering
        specularBounce = bs->IsSpecular();
        anyNonSpecularBounces |= !bs->IsSpecular();
        if (bs->IsTransmission())
            etaScale *= Sqr(bs->eta);
        ray = isect.SpawnRay(ray, bsdf, bs->wi, bs->flags, bs->eta);

        // Account for attenuated subsurface scattering, if applicable
        BSSRDF bssrdf = isect.GetBSSRDF(ray, lambda, camera, scratchBuffer);
        if (bssrdf && bs->IsTransmission()) {
            // Sample BSSRDF probe segment to find exit point
            Float uc = sampler.Get1D();
            Point2f up = sampler.Get2D();
            pstd::optional<BSSRDFProbeSegment> probeSeg = bssrdf.SampleSp(uc, up);
            if (!probeSeg)
                break;

            // Sample random intersection along BSSRDF probe segment
            uint64_t seed = MixBits(FloatToBits(sampler.Get1D()));
            WeightedReservoirSampler<SubsurfaceInteraction> interactionSampler(seed);
            // Intersect BSSRDF sampling ray against the scene geometry
            Interaction base(probeSeg->p0, ray.time, Medium());
            while (true) {
                Ray r = base.SpawnRayTo(probeSeg->p1);
                if (r.d == Vector3f(0, 0, 0))
                    break;
                pstd::optional<ShapeIntersection> ssi = Intersect(r, 1);
                if (!ssi)
                    break;
                base = ssi->intr; // NOLINT(*-slicing)
                if (ssi->intr.material == isect.material)
                    interactionSampler.Add(SubsurfaceInteraction(ssi->intr), 1.f);
            }

            if (!interactionSampler.HasSample())
                break;

            // Convert probe intersection to _BSSRDFSample_
            SubsurfaceInteraction sssi = interactionSampler.GetSample();
            BSSRDFSample bssrdfSample =
                    bssrdf.ProbeIntersectionToSample(sssi, scratchBuffer);
            if (!bssrdfSample.Sp || !bssrdfSample.pdf)
                break;

            // Update path state for subsurface scattering
            Float pdf = interactionSampler.SampleProbability() * bssrdfSample.pdf[0];
            beta *= bssrdfSample.Sp / pdf;
            r_u *= bssrdfSample.pdf / bssrdfSample.pdf[0];
            SurfaceInteraction pi = sssi;
            pi.wo = bssrdfSample.wo;
            prevIntrContext = LightSampleContext(pi);
            // Possibly regularize subsurface BSDF
            BSDF& Sw = bssrdfSample.Sw;
            anyNonSpecularBounces = true;
            if (regularize) {
                ++regularizedBSDFs;
                Sw.Regularize();
            } else
                ++totalBSDFs;

            // Account for attenuated direct illumination subsurface scattering
            L += SampleLd(pi, &Sw, lambda, sampler, beta, r_u);

            // Sample ray for indirect subsurface scattering
            Float ssu = sampler.Get1D();
            pstd::optional<BSDFSample> ssbs = Sw.Sample_f(pi.wo, ssu, sampler.Get2D());
            if (!ssbs)
                break;
            beta *= ssbs->f * AbsDot(ssbs->wi, pi.shading.n) / ssbs->pdf;
            r_l = r_u / ssbs->pdf;
            // Don't increment depth this time...
            DCHECK(!IsInf(beta.y(lambda)));
            specularBounce = ssbs->IsSpecular();
            ray = RayDifferential(pi.SpawnRay(ssbs->wi));
        }

        // Possibly terminate volumetric path with Russian roulette
        if (!beta)
            break;
        SampledSpectrum rrBeta = beta * etaScale / r_u.Average();
        Float uRR = sampler.Get1D();
        PBRT_DBG("%s\n",
                 StringPrintf("etaScale %f -> rrBeta %s", etaScale, rrBeta).c_str());
        if (rrBeta.MaxComponentValue() < 1 && depth > 1) {
            Float q = std::max<Float>(0, 1 - rrBeta.MaxComponentValue());
            if (uRR < q)
                break;
            beta /= 1 - q;
        }
    }
    return L;
}

void GraphVolPathIntegrator::VolTrace() {

}

SampledSpectrum GraphVolPathIntegrator::SampleLd(const Interaction& intr, const BSDF* bsdf,
                                            SampledWavelengths& lambda, Sampler sampler,
                                            SampledSpectrum beta,
                                            SampledSpectrum r_p) const {
    // Estimate light-sampled direct illumination at _intr_
    // Initialize _LightSampleContext_ for volumetric light sampling
    LightSampleContext ctx;
    if (bsdf) {
        ctx = LightSampleContext(intr.AsSurface());
        // Try to nudge the light sampling position to correct side of the surface
        BxDFFlags flags = bsdf->Flags();
        if (IsReflective(flags) && !IsTransmissive(flags))
            ctx.pi = intr.OffsetRayOrigin(intr.wo);
        else if (IsTransmissive(flags) && !IsReflective(flags))
            ctx.pi = intr.OffsetRayOrigin(-intr.wo);

    } else
        ctx = LightSampleContext(intr);

    // Sample a light source using _lightSampler_
    Float u = sampler.Get1D();
    pstd::optional<SampledLight> sampledLight = lightSampler.Sample(ctx, u);
    Point2f uLight = sampler.Get2D();
    if (!sampledLight)
        return SampledSpectrum(0.f);
    Light light = sampledLight->light;
    DCHECK(light && sampledLight->p != 0);

    // Sample a point on the light source
    pstd::optional<LightLiSample> ls = light.SampleLi(ctx, uLight, lambda, true);
    if (!ls || !ls->L || ls->pdf == 0)
        return SampledSpectrum(0.f);
    Float p_l = sampledLight->p * ls->pdf;

    // Evaluate BSDF or phase function for light sample direction
    Float scatterPDF;
    SampledSpectrum f_hat;
    Vector3f wo = intr.wo, wi = ls->wi;
    if (bsdf) {
        // Update _f_hat_ and _scatterPDF_ accounting for the BSDF
        f_hat = bsdf->f(wo, wi) * AbsDot(wi, intr.AsSurface().shading.n);
        scatterPDF = bsdf->PDF(wo, wi);

    } else {
        // Update _f_hat_ and _scatterPDF_ accounting for the phase function
        CHECK(intr.IsMediumInteraction());
        PhaseFunction phase = intr.AsMedium().phase;
        f_hat = SampledSpectrum(phase.p(wo, wi));
        scatterPDF = phase.PDF(wo, wi);
    }
    if (!f_hat)
        return SampledSpectrum(0.f);

    // Declare path state variables for ray to light source
    Ray lightRay = intr.SpawnRayTo(ls->pLight);
    SampledSpectrum T_ray(1.f), r_l(1.f), r_u(1.f);
    RNG rng(Hash(lightRay.o), Hash(lightRay.d));

    while (lightRay.d != Vector3f(0, 0, 0)) {
        // Trace ray through media to estimate transmittance
        pstd::optional<ShapeIntersection> si = Intersect(lightRay, 1 - ShadowEpsilon);
        // Handle opaque surface along ray's path
        if (si && si->intr.material)
            return SampledSpectrum(0.f);

        // Update transmittance for current ray segment
        if (lightRay.medium) {
            Float tMax = si ? si->tHit : (1 - ShadowEpsilon);
            Float mediumU = rng.Uniform<Float>();
            SampledSpectrum T_maj =
                    SampleT_maj(lightRay, tMax, mediumU, rng, lambda,
                                [&](Point3f p, MediumProperties& mp, SampledSpectrum sigma_maj,
                                    SampledSpectrum T_maj) {
                                    // Update ray transmittance estimate at sampled point
                                    // Update _T_ray_ and PDFs using ratio-tracking estimator
                                    SampledSpectrum sigma_n =
                                            ClampZero(sigma_maj - mp.sigma_a - mp.sigma_s);
                                    Float pdf = T_maj[0] * sigma_maj[0];
                                    T_ray *= T_maj * sigma_n / pdf;
                                    r_l *= T_maj * sigma_maj / pdf;
                                    r_u *= T_maj * sigma_n / pdf;

                                    // Possibly terminate transmittance computation using
                                    // Russian roulette
                                    SampledSpectrum Tr = T_ray / (r_l + r_u).Average();
                                    if (Tr.MaxComponentValue() < 0.05f) {
                                        Float q = 0.75f;
                                        if (rng.Uniform<Float>() < q)
                                            T_ray = SampledSpectrum(0.);
                                        else
                                            T_ray /= 1 - q;
                                    }

                                    if (!T_ray)
                                        return false;
                                    return true;
                                });
            // Update transmittance estimate for final segment
            T_ray *= T_maj / T_maj[0];
            r_l *= T_maj / T_maj[0];
            r_u *= T_maj / T_maj[0];
        }

        // Generate next ray segment or return final transmittance
        if (!T_ray)
            return SampledSpectrum(0.f);
        if (!si)
            break;
        lightRay = si->intr.SpawnRayTo(ls->pLight);
    }
    // Return path contribution function estimate for direct lighting
    r_l *= r_p * p_l;
    r_u *= r_p * scatterPDF;
    if (IsDeltaLight(light.Type()))
        return beta * f_hat * T_ray * ls->L / r_l.Average();
    else
        return beta * f_hat * T_ray * ls->L / (r_l + r_u).Average();
}

std::string GraphVolPathIntegrator::ToString() const {
    return StringPrintf(
            "[ GraphVolPathIntegrator maxDepth: %d lightSampler: %s regularize: %s ]", maxDepth,
            lightSampler, regularize);
}

std::unique_ptr<GraphVolPathIntegrator> GraphVolPathIntegrator::Create(
        const ParameterDictionary& parameters, Camera camera, Sampler sampler,
        Primitive aggregate, std::vector<Light> lights)
{
    int maxDepth = parameters.GetOneInt("maxdepth", 5);
    std::string lightStrategy = parameters.GetOneString("lightsampler", "bvh");
    bool regularize = parameters.GetOneBool("regularize", false);

    return std::make_unique<GraphVolPathIntegrator>(
            maxDepth, std::move(camera), std::move(sampler), std::move(aggregate), std::move(lights),
            lightStrategy, regularize);
}

}
