#include "GCore/Components/MeshOperand.h"
#include "GCore/util_openmesh_bind.h"
#include "geom_node_base.h"
#include <cmath>
#include <Eigen/Sparse>
#include <map>
#include <vector>
#include <queue>
#include "../utils/min_surface.h"

NODE_DEF_OPEN_SCOPE
NODE_DECLARATION_FUNCTION(min_surface)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<Geometry>("Input");

    // Output-1: Minimal surface with fixed boundary
    b.add_output<Geometry>("Output");
}

NODE_EXECUTION_FUNCTION(min_surface)
{
    // Get the input from params
    auto input = params.get_input<Geometry>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Minimal Surface: Need Geometry Input.");
        return false;
    }

    auto halfedge_mesh = operand_to_openmesh(&input);

    set_min_surface(uniform_weights, halfedge_mesh, halfedge_mesh);

    auto geometry = openmesh_to_operand(halfedge_mesh.get());
    
    // Set the output of the nodes
    params.set_output("Output", std::move(*geometry));
    return true;
}

NODE_DECLARATION_UI(min_surface);

NODE_DECLARATION_FUNCTION(min_surface_cotangent_weights)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<Geometry>("Input");
    b.add_input<Geometry>("Reference Mesh");
    b.add_output<Geometry>("Output");
}

NODE_EXECUTION_FUNCTION(min_surface_cotangent_weights)
{
    // Get the input from params
    auto ref_input = params.get_input<Geometry>("Reference Mesh");
    auto input = params.get_input<Geometry>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Minimal Surface: Need Geometry Input.");
        return false;
    }
    auto ref_halfedge_mesh = operand_to_openmesh(&ref_input);
    auto halfedge_mesh = operand_to_openmesh(&input);

    set_min_surface(cotangent_weights,ref_halfedge_mesh,halfedge_mesh);

    auto geometry = openmesh_to_operand(halfedge_mesh.get());
    
    // Set the output of the nodes
    params.set_output("Output", std::move(*geometry));
    return true;
}

NODE_DECLARATION_UI(min_surface_cotangent_weights);

NODE_DECLARATION_FUNCTION(min_surface_shape_preserving)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<Geometry>("Input");
    b.add_input<Geometry>("Reference Mesh");
    b.add_output<Geometry>("Output");
}

NODE_EXECUTION_FUNCTION(min_surface_shape_preserving)
{
    // Get the input from params
    auto ref_input = params.get_input<Geometry>("Reference Mesh");
    auto input = params.get_input<Geometry>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Minimal Surface: Need Geometry Input.");
        return false;
    }
    auto ref_halfedge_mesh = operand_to_openmesh(&ref_input);
    auto halfedge_mesh = operand_to_openmesh(&input);

    set_min_surface(shape_preserving_weights,ref_halfedge_mesh,halfedge_mesh);

    auto geometry = openmesh_to_operand(halfedge_mesh.get());
    
    // Set the output of the nodes
    params.set_output("Output", std::move(*geometry));
    return true;
}

NODE_DECLARATION_UI(min_surface_shape_preserving);

NODE_DEF_CLOSE_SCOPE
