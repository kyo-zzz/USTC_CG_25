#include "GCore/Components/MeshOperand.h"
#include "geom_node_base.h"
#include "GCore/util_openmesh_bind.h"
#include <Eigen/Sparse>
#include <iostream>
#include <memory>
#include <cmath>


NODE_DEF_OPEN_SCOPE

NODE_DECLARATION_FUNCTION(circle_boundary_mapping)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<Geometry>("Input");
    // Output-1: Processed 3D mesh whose boundary is mapped to a square and the
    // interior vertices remains the same
    b.add_output<Geometry>("Output");
}

NODE_EXECUTION_FUNCTION(circle_boundary_mapping)
{
    auto input = params.get_input<Geometry>("Input");

    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Boundary Mapping: Need Geometry Input.");
    }

    const auto& mesh = operand_to_openmesh(&input);
    std::vector<std::unique_ptr<OpenMesh::SmartVertexHandle>> boundary_vertices;
    
    // Find first boundary vertex
    OpenMesh::SmartVertexHandle boundary_start, current, previous;
    for (auto& vertex : mesh->vertices()) {
        if (vertex.is_boundary()) {
            boundary_start = vertex;
            break;
        }
    }

    // Traverse boundary and store cumulative lengths
    double total_length = 0.0;
    std::vector<double> cumulative_lengths;
    current = previous = boundary_start;

    do {
        for (const auto& halfedge : current.outgoing_halfedges()) {
            auto next = halfedge.to();
            if (next.is_boundary() && next != previous) {
                boundary_vertices.push_back(std::make_unique<OpenMesh::SmartVertexHandle>(current));
                cumulative_lengths.push_back(total_length);
                total_length += (mesh->point(next) - mesh->point(current)).length();
                previous = current;
                current = next;
                break;
            }
        }
    } while (current != boundary_start);

    // Map vertices to circle boundary
    const double TWO_PI = 2.0 * 3.14159265358979323846;
    for (size_t i = 0; i < boundary_vertices.size(); ++i) {
        double angle = TWO_PI * cumulative_lengths[i] / total_length;
        auto& point = mesh->point(*(boundary_vertices[i]));
        point[0] = std::cos(angle);
        point[1] = std::sin(angle);
        point[2] = 0.0;
    }

    auto geometry = openmesh_to_operand(mesh.get());
    params.set_output("Output", std::move(*geometry));
    return true;
}

NODE_DECLARATION_FUNCTION(square_boundary_mapping)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<Geometry>("Input");

    // Output-1: Processed 3D mesh whose boundary is mapped to a square and the
    // interior vertices remains the same
    b.add_output<Geometry>("Output");
}

NODE_EXECUTION_FUNCTION(square_boundary_mapping) {
    auto input = params.get_input<Geometry>("Input");
    
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Input does not contain a mesh");
    }

    // Get mesh and find first boundary vertex
    const auto& mesh = operand_to_openmesh(&input);
    std::vector<std::unique_ptr<OpenMesh::SmartVertexHandle>> boundary_vertices;
    OpenMesh::SmartVertexHandle boundary_start, current, previous;

    // Find first boundary vertex
    for (auto& vertex : mesh->vertices()) {
        if (vertex.is_boundary()) {
            boundary_start = vertex;
            break;
        }
    }

    // Traverse boundary and calculate cumulative lengths
    double total_length = 0.0;
    std::vector<double> cumulative_lengths;
    current = previous = boundary_start;

    do {
        for (const auto& halfedge : current.outgoing_halfedges()) {
            auto next = halfedge.to();
            if (next.is_boundary() && next != previous) {
                boundary_vertices.push_back(std::make_unique<OpenMesh::SmartVertexHandle>(current));
                cumulative_lengths.push_back(total_length);
                total_length += (mesh->point(next) - mesh->point(current)).length();
                previous = current;
                current = next;
                break;
            }
        }
    } while (current != boundary_start);

    // Map boundary vertices to square
    for (size_t i = 0; i < boundary_vertices.size(); ++i) {
        float x, y;
        double normalized_length = cumulative_lengths[i] / total_length;
        double t_x, t_y;
        
        // Handle corner cases
        if (i == 0 || static_cast<int>(normalized_length * 4) > 
                      static_cast<int>(cumulative_lengths[i-1] / total_length * 4)) {
            t_x = static_cast<double>(static_cast<int>(normalized_length * 4)) / 4;
            t_y = static_cast<double>(static_cast<int>(normalized_length * 4 + 1)) / 4;
        } else {
            t_x = normalized_length;
            t_y = normalized_length + 0.25;
        }

        // Map x coordinate
        t_x -= static_cast<double>(static_cast<int>(t_x));
        if (t_x < 0.25) x = 0.0;
        else if (t_x < 0.50) x = t_x * 4.0 - 1.0;
        else if (t_x < 0.75) x = 1.0;
        else x = 4.0 - t_x * 4.0;

        // Map y coordinate
        t_y -= static_cast<double>(static_cast<int>(t_y));
        if (t_y < 0.25) y = 0.0;
        else if (t_y < 0.50) y = t_y * 4.0 - 1.0;
        else if (t_y < 0.75) y = 1.0;
        else y = 4.0 - t_y * 4.0;
        
        // Update vertex position
        auto& point = mesh->point(*(boundary_vertices[i]));
        point[0] = x;
        point[1] = y;
        point[2] = 0.0;
    }

    // Create output geometry
    auto geometry = openmesh_to_operand(mesh.get());
    params.set_output("Output", std::move(*geometry));
    return true;
}


NODE_DECLARATION_UI(boundary_mapping);
NODE_DEF_CLOSE_SCOPE
