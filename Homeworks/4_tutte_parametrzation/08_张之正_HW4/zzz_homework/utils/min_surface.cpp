#include "min_surface.h"
#include <queue>
#include <iostream>

// Weight function implementation
float uniform_weights(const OpenMesh::Vec3f&, const OpenMesh::Vec3f&, const OpenMesh::Vec3f&) {
    return 1.0f;
}

float shape_preserving_weights(const OpenMesh::Vec3f& vec0, 
                             const OpenMesh::Vec3f& vecl, 
                             const OpenMesh::Vec3f& vecr) {
    return (vecl.cross(vec0).length() / (vec0.sqrnorm() * vecl.length())) +
           (vecr.cross(vec0).length() / (vec0.sqrnorm() * vecr.length()));
}

float cotangent_weights(const OpenMesh::Vec3f& vec0, 
                       const OpenMesh::Vec3f& vecl, 
                       const OpenMesh::Vec3f& vecr) {
    return ((vecl - vec0).dot(vecl) / vecl.cross(vec0).length()) +
           ((vecr - vec0).dot(vecr) / vecr.cross(vec0).length());
}

void set_min_surface(float(*weight)(const OpenMesh::Vec3f&, const OpenMesh::Vec3f&, const OpenMesh::Vec3f&),
                    std::shared_ptr<USTC_CG::PolyMesh> reference_mesh,
                    std::shared_ptr<USTC_CG::PolyMesh> mesh) {
    // Initialize solver and matrix
    auto [coefficient_matrix, solver_] = std::pair<Eigen::SparseMatrix<double>, 
                                                 Eigen::SparseLU<Eigen::SparseMatrix<double>>>{};
    std::array<Eigen::VectorXd, 3> constraints;
    std::array<Eigen::VectorXd, 3> solution_;
    
    // Define vertex mapping
    std::map<int, int> vertex_mapping;
    std::vector<int> reverse_mapping;

    // Build mapping for non-boundary vertices
    for (const auto& vertex_handle : reference_mesh->vertices()) {
        if (!vertex_handle.is_boundary()) {
            vertex_mapping[vertex_handle.idx()] = static_cast<int>(reverse_mapping.size());
            reverse_mapping.push_back(vertex_handle.idx());
        }
    }

    // Initialize constraint vectors
    for (auto& constraint : constraints) {
        constraint = Eigen::VectorXd::Zero(reverse_mapping.size());
    }

    // Build coefficient matrix
    std::vector<Eigen::Triplet<double>> tripletList;
    coefficient_matrix.resize(reverse_mapping.size(), reverse_mapping.size());

    // Iterate through each vertex for calculation
    for (const auto& vertex_handle : reference_mesh->vertices()) {
        if (vertex_handle.is_boundary()) continue;

        double sum_weight = 0.0;
        std::queue<double> weights;
        const auto& position = reference_mesh->point(vertex_handle);

        // Calculate weights for adjacent vertices
        for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges()) {
            const auto& [v0, vl, vr] = std::tuple{
                halfedge_handle.to(),
                halfedge_handle.prev().opp().to(),
                halfedge_handle.opp().prev().prev().to()
            };
            
            const auto vec0 = reference_mesh->point(v0) - position;
            const auto vecl = reference_mesh->point(vl) - position;
            const auto vecr = reference_mesh->point(vr) - position;

            double w = weight(vec0, vecl, vecr);
            sum_weight += w;
            weights.push(w);
        }

        // Update coefficient matrix and constraint vectors
        for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges()) {
            double w = weights.front();
            weights.pop();

            if (halfedge_handle.to().is_boundary()) {
                for (int j = 0; j < 3; ++j) {
                    constraints[j](vertex_mapping[vertex_handle.idx()]) += 
                        mesh->point(halfedge_handle.to())[j] * w / sum_weight;
                }
            } else {
                tripletList.emplace_back(
                    vertex_mapping[vertex_handle.idx()],
                    vertex_mapping[halfedge_handle.to().idx()],
                    -w / sum_weight
                );
            }
        }

        tripletList.emplace_back(
            vertex_mapping[vertex_handle.idx()],
            vertex_mapping[vertex_handle.idx()],
            1.0
        );
    }

    // Solve linear system
    coefficient_matrix.setFromTriplets(tripletList.begin(), tripletList.end());
    solver_.compute(coefficient_matrix);

    // Update mesh vertex positions
    for (int j = 0; j < 3; ++j) {
        solution_[j] = solver_.solve(constraints[j]);
        for (const auto& vertex_handle : mesh->vertices()) {
            if (!vertex_handle.is_boundary()) {
                mesh->point(vertex_handle)[j] = solution_[j](vertex_mapping[vertex_handle.idx()]);
            }
        }
    }
}
