#include "render_node_base.h"
#include "..\light.h"
#include "..\geometries\mesh.h"
#include "nodes\core\def\node_def.hpp"
#include "pxr\base\gf\frustum.h"
#include "pxr\imaging\glf\simpleLight.h"
#include "pxr\imaging\hd\tokens.h"

NODE_DEF_OPEN_SCOPE
    NODE_DECLARATION_FUNCTION(shadow_mapping) {
        b.add_input<int>("resolution")
            .default_val(1024)
            .min(256)
            .max(4096);
        b.add_input<std::string>("Shader").default_val("shaders/shadow_mapping.fs");

        b.add_output<TextureHandle>("Shadow Maps");
    }

    NODE_EXECUTION_FUNCTION(shadow_mapping) {
        auto resolution = params.get_input<int>("resolution");

        TextureDesc texture_desc;
        texture_desc.array_size = lights.size() * 6;
        texture_desc.size = GfVec2i(resolution);
        texture_desc.format = HdFormatUNorm8Vec4;
        auto shadow_map_texture = resource_allocator.create(texture_desc);

        auto shaderPath = params.get_input<std::string>("Shader");

        ShaderDesc shader_desc;
        shader_desc.set_vertex_path(
            std::filesystem::path(RENDER_NODES_FILES_DIR) /
            std::filesystem::path("shaders/shadow_mapping.vs")
        );
        shader_desc.set_fragment_path(
            std::filesystem::path(RENDER_NODES_FILES_DIR) /
            std::filesystem::path(shaderPath)
        );
        auto shader_handle = resource_allocator.create(shader_desc);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        std::vector<TextureHandle> depth_textures;
        GLuint framebuffer;
        glGenFramebuffers(1, &framebuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

        GfFrustum frustum;
        frustum.SetPerspective(90, 1, 0.1, 25);
        const auto light_projection_mat = GfMatrix4f(frustum.ComputeProjectionMatrix());
        static constexpr GfVec3f directions[] = {
            {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {-1, 0, 0}, {0, -1, 0}, {0, 0, -1}
        };

        glViewport(0, 0, resolution, resolution);
        int NN = lights.size();
        auto tmp = global_payload;
        for(int light_id = 0; light_id < NN; ++light_id) {
            if(lights[light_id]->GetId().IsEmpty() || lights[light_id]->GetLightType() != HdPrimTypeTokens->sphereLight) {
                continue;
            }

            shader_handle->shader.use();
            auto light_params = lights[light_id]->Get(HdTokens->params).Get<GlfSimpleLight>();
            GfVec3f light_position = {
                light_params.GetPosition()[0],
                light_params.GetPosition()[1],
                light_params.GetPosition()[2]
            };

            for(int i = 0; i < 6; i++) {
                const auto light_view_mat = GfMatrix4f().SetLookAt(light_position, light_position + directions[i], directions[(i + 1) % 6]);
                shader_handle->shader.setMat4("light_view", light_view_mat);
                shader_handle->shader.setMat4("light_projection", GfMatrix4f(light_projection_mat));

                glFramebufferTextureLayer(
                    GL_FRAMEBUFFER,
                    GL_COLOR_ATTACHMENT0,
                    shadow_map_texture->texture_id,
                    0,
                    light_id * 6 + i
                );

                texture_desc.format = HdFormatFloat32UInt8;
                texture_desc.array_size = 1;
                auto depth_texture_for_opengl = resource_allocator.create(texture_desc);
                depth_textures.push_back(depth_texture_for_opengl);

                glFramebufferTexture2D(
                    GL_FRAMEBUFFER,
                    GL_DEPTH_STENCIL_ATTACHMENT,
                    GL_TEXTURE_2D,
                    depth_texture_for_opengl->texture_id,
                    0
                );

                glClearColor(0.f, 0.f, 0.f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

                for(int mesh_id = 0; mesh_id < meshes.size(); ++mesh_id) {
                    auto mesh = meshes[mesh_id];

                    shader_handle->shader.setMat4("model", mesh->transform);
                    mesh->RefreshGLBuffer();

                    glBindVertexArray(mesh->VAO);
                    glDrawElements(
                        GL_TRIANGLES,
                        static_cast<unsigned int>(mesh->triangulatedIndices.size() * 3),
                        GL_UNSIGNED_INT,
                        nullptr
                    );
                    glBindVertexArray(0);
                }
            }
        }

        for(auto &&depth_texture: depth_textures) {
            resource_allocator.destroy(depth_texture);
        }

        resource_allocator.destroy(shader_handle);
        glDeleteFramebuffers(1, &framebuffer);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        auto shader_error = shader_handle->shader.get_error();

        params.set_output("Shadow Maps", shadow_map_texture);
        if(!shader_error.empty()) {
            throw std::runtime_error(shader_error);
        }
    }

    NODE_DECLARATION_UI(shadow_mapping);
NODE_DEF_CLOSE_SCOPE
