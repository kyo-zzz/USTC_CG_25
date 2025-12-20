#include "path.h"

#include <random>

#include "../surfaceInteraction.h"
USTC_CG_NAMESPACE_OPEN_SCOPE
using namespace pxr;

VtValue PathIntegrator::Li(const GfRay& ray,std::default_random_engine& random)
{
    std::uniform_real_distribution<float> uniform_dist(
        0.0f,1.0f - std::numeric_limits<float>::epsilon());
    std::function<float()> uniform_float = std::bind(uniform_dist,random);

    auto color = EstimateOutGoingRadiance(ray,uniform_float,0);

    return VtValue(GfVec3f(color[0],color[1],color[2]));
}
/*
 * TODO: You need to complete this function to achieve the estimate of the outgoing Radiance
 * */
// Estimate the outgoing radiance along a ray using path tracing
GfVec3f PathIntegrator::EstimateOutGoingRadiance(
    const GfRay& ray,
    const std::function<float()>& uniform_float,
    int recursion_depth)
{
    // Limit recursion depth to avoid infinite loops
    if(recursion_depth >= 50) {
        return GfVec3f(0);
    }

    // Russian roulette termination for path tracing
    const float continuation_probability = 0.8f;
    if(uniform_float() > continuation_probability) {
        return GfVec3f(0);
    }

    SurfaceInteraction si;
    // Intersect the ray with the scene
    if(!Intersect(ray,si)) {
        // If no intersection, return environment light for primary ray
        return (recursion_depth == 0) ? IntersectDomeLight(ray) : GfVec3f(0);
    }

    GfVec3f intersectPos;
    // Check if the ray hits a light source
    GfVec3f light_color = IntersectLights(ray,intersectPos);
    if(light_color != GfVec3f(0)) {
        return light_color;
    }

    // Ensure the shading normal is oriented correctly
    if(GfDot(si.shadingNormal,ray.GetDirection()) > 0) {
        si.flipNormal();
        si.PrepareTransforms();
    }

    // Estimate direct lighting at the intersection point
    GfVec3f radiance = EstimateDirectLight(si,uniform_float);
    GfVec3f wo = -GfVec3f(ray.GetDirection().GetNormalized());

    GfVec3f wi;
    float pdf = 1.0f;
    // Sample the BRDF to get a new direction
    Color brdf = si.material->Sample(wo,wi,pdf,si.texcoord,uniform_float);

    // If the sampled direction is valid, compute indirect lighting
    if(pdf > 1e-6f) {
        GfRay new_ray(si.position,wi);
        // Recursively estimate radiance along the new ray
        GfVec3f indirect = EstimateOutGoingRadiance(new_ray,uniform_float,recursion_depth + 1);
        float cos_theta = std::max(0.0f,GfDot(wi,si.shadingNormal));

        // Compute the contribution from indirect lighting
        GfVec3f global_light = GfVec3f(
            brdf[0] * indirect[0],
            brdf[1] * indirect[1],
            brdf[2] * indirect[2]
        );

        global_light *= cos_theta / (pdf * continuation_probability);
        radiance += global_light;
    }

    // Clamp the radiance to avoid overflow
    return GfVec3f(
        std::min(radiance[0],255.0f),
        std::min(radiance[1],255.0f),
        std::min(radiance[2],255.0f)
    );
}
USTC_CG_NAMESPACE_CLOSE_SCOPE
