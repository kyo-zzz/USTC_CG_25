#version 430 core

#define SAMPLES 25

// Light structure for uniform buffer
struct Light {
    mat4 light_projection;
    mat4 light_view;
    vec3 position;
    float radius;
    vec3 color;
    int shadow_map_id;
};

// Buffer for multiple lights
layout(binding = 0) buffer lightsBuffer {
    Light lights[6];
};

uniform vec2 iResolution;

uniform sampler2D diffuseColorSampler;
uniform sampler2D normalMapSampler; // Normal mapping applied in rasterize_impl.fs
uniform sampler2D metallicRoughnessSampler;
uniform sampler2DArray shadow_maps;
uniform sampler2D position;

uniform vec3 camPos;
uniform int light_count;

layout(location = 0) out vec4 Color;

// Find average blocker depth in a region around the receiver
float findBlockerDistance(vec2 coords, float receiverDepth, int size, int shadow_map_id) {
    float blockerSum = 0.0;
    int blockerCount = 0;
    vec2 texelSize = 1.0 / textureSize(shadow_maps, 0).xy;
    for (int i = -size; i <= size; ++i) {
        for (int j = -size; j <= size; ++j) {
            float depth = texture(shadow_maps, vec3(coords + vec2(i, j) * texelSize, shadow_map_id)).x;
            if (depth < receiverDepth) {
                blockerSum += depth;
                blockerCount++;
            }
        }
    }
    return blockerCount > 0 ? blockerSum / blockerCount : -2.0;
}

// Percentage Closer Filtering (PCF) for soft shadows
float pcf(vec2 coord, float depth, float size, int shadow_map_id) {
    const float ANGLE_STEP = 2*3.1415926 / SAMPLES;
    const float RADIUS_STEP = size / SAMPLES;
    float angle = 2*3.1415926 * fract(dot(coord, vec2(12.9898, 78.233)) * 43758.545312);
    float radius = RADIUS_STEP;
    vec2 texelSize = 1.0 / textureSize(shadow_maps, 0).xy;
    float coef = 0.0;
    for (int i = 0; i < SAMPLES; i++) {
        vec2 offset = vec2(cos(angle), sin(angle)) * pow(radius, 0.75);
        float depthSample = texture(shadow_maps, vec3(coord + offset * texelSize, shadow_map_id)).x;
        coef += depth <= depthSample ? 1.0 : 0.0;
        angle += ANGLE_STEP;
        radius += RADIUS_STEP;
    }
    return coef / SAMPLES;
}

void main() {
    vec2 uv = gl_FragCoord.xy / iResolution;

    // Fetch position and normal from G-buffer
    vec3 pos = texture(position, uv).xyz;
    vec3 normal = texture(normalMapSampler, uv).xyz * 2.0 - 1.0;

    // Fetch material properties
    vec4 metalnessRoughness = texture(metallicRoughnessSampler, uv);
    float metal = metalnessRoughness.x;
    float roughness = metalnessRoughness.y;
    vec3 material = texture(diffuseColorSampler, uv).xyz;

    // Ambient term
    Color = vec4(material * 0.1, 1.0);

    // Loop over all lights
    for (int i = 0; i < light_count; i++) {
        // Transform position to light space
        vec4 p = lights[i].light_projection * lights[i].light_view * vec4(pos, 1.0);
        p /= p.w;
        vec2 shadow_map_uv = p.xy * 0.5 + 0.5;

        // Skip if outside light frustum
        if (p.x < -1.0 || p.x > 1.0 || p.y < -1.0 || p.y > 1.0) {
            continue;
        }

        // Lighting calculations
        vec3 lightDir = lights[i].position - pos;
        float dist = length(lightDir);
        lightDir /= dist;

        float shadow_map_value = texture(shadow_maps, vec3(shadow_map_uv, lights[i].shadow_map_id)).x;
        float depth = p.z - max(0.05 * (1.0 - dot(normal, lightDir)), 0.005);

        // Shadow test
        if (depth > shadow_map_value) {
            continue;
        }

        // Blinn-Phong shading
        vec3 viewDir = normalize(camPos - pos);
        vec3 halfDir = normalize(lightDir + viewDir);
        vec3 diffuse = lights[i].color * material * max(dot(normal, lightDir), 0.0);
        vec3 specular = lights[i].color * pow(max(dot(normal, halfDir), 0.0), 64.0);

        // PCSS soft shadow
        float blockerDistance = findBlockerDistance(shadow_map_uv, depth, 2, lights[i].shadow_map_id);
        if (blockerDistance > -1.0) {
            float penumbraRatio = (p.z - blockerDistance) * lights[i].radius / blockerDistance;
            float shadow = pcf(shadow_map_uv, p.z, 7.0 * lights[i].radius * penumbraRatio, lights[i].shadow_map_id);
            Color += vec4((diffuse + specular) * shadow, 1.0);
        } else {
            Color += vec4(diffuse + specular, 1.0);
        }
    }
}