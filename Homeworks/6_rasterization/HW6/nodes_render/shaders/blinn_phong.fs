#version 430 core

// Light structure definition
struct Light {
    mat4 light_projection;
    mat4 light_view;
    vec3 position;
    float radius;
    vec3 color;
    int shadow_map_id;
};

// Buffer for multiple lights (up to 6)
layout(binding = 0) buffer lightsBuffer {
    Light lights[6];
};

// Screen resolution
uniform vec2 iResolution;

// Texture samplers
uniform sampler2D diffuseColorSampler;
uniform sampler2D normalMapSampler; // Normal mapping applied in rasterize_impl.fs
uniform sampler2D metallicRoughnessSampler;
uniform sampler2DArray shadow_maps;
uniform sampler2D position;

// Camera position
uniform vec3 camPos;

// Number of active lights
uniform int light_count;

// Output color
layout(location = 0) out vec4 Color;

void main() {
    // Calculate UV coordinates from fragment position
    vec2 uv = gl_FragCoord.xy / iResolution;

    // Fetch position and normal from G-buffer
    vec3 pos = texture(position, uv).xyz;
    vec3 normal = texture(normalMapSampler, uv).xyz * 2.0 - 1.0;

    // Fetch metallic and roughness values
    vec4 metalnessRoughness = texture(metallicRoughnessSampler, uv);
    float metal = metalnessRoughness.x;
    float roughness = metalnessRoughness.y;

    // Fetch base color (albedo)
    vec3 material = texture(diffuseColorSampler, uv).xyz;

    // Initialize color with ambient term
    Color = vec4(material * 0.1, 1.0);

    // Loop over all lights
    for (int i = 0; i < light_count; ++i) {
        // Transform position to light's clip space
        vec4 p = lights[i].light_projection * lights[i].light_view * vec4(pos, 1.0);
        p /= p.w;

        // Skip if outside light's frustum
        if (p.x < -1.0 || p.x > 1.0 || p.y < -1.0 || p.y > 1.0) {
            continue;
        }

        // Compute light direction and distance
        vec3 lightDir = lights[i].position - pos;
        float dist = length(lightDir);
        lightDir /= dist;

        // Shadow mapping: sample shadow map
        float shadow_map_value = texture(
            shadow_maps, 
            vec3(p.xy * 0.5 + 0.5, lights[i].shadow_map_id)
        ).x;

        // Shadow test
        if (p.z - max(0.05 * (1.0 - dot(normal, lightDir)), 0.005) > shadow_map_value) {
            continue;
        }

        // Compute Blinn-Phong lighting
        vec3 viewDir = normalize(camPos - pos);
        vec3 halfDir = normalize(lightDir + viewDir);

        // Diffuse term
        vec3 diffuse = lights[i].color * material * max(dot(normal, lightDir), 0.0);

        // Specular term (hardcoded shininess)
        vec3 specular = lights[i].color * pow(max(dot(normal, halfDir), 0.0), 64.0);

        // Accumulate lighting
        Color += vec4(diffuse + specular, 1.0);
    }
}