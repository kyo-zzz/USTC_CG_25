#include "GCore/Components/MeshOperand.h"
#include "GCore/util_openmesh_bind.h"
#include "geom_node_base.h"
#include <cmath>
#include <ctime>
#include <Eigen/Dense>
#include <Eigen/SparseLU>
#include <Eigen/Sparse>
#include <map>
#include <vector>
#include <set>
#include <algorithm>
#include "../utils/arap.h"

NODE_DEF_OPEN_SCOPE

// Node declaration: ARAP deformation node
NODE_DECLARATION_FUNCTION(arap)
{
    b.add_input<Geometry>("Input");                  
    b.add_input<Geometry>("Reference Mesh");         
    b.add_input<int>("T").default_val(10).min(0).max(20);         
    b.add_input<int>("If asap").default_val(0).min(0).max(1);    
    b.add_output<pxr::VtArray<pxr::GfVec2f>>("OutputUV"); 
}

// Node execution: runs ARAP algorithm
NODE_EXECUTION_FUNCTION(arap)
{
    auto input = params.get_input<Geometry>("Input");
    auto ref_input = params.get_input<Geometry>("Reference Mesh");
    auto T = params.get_input<int>("T");
    auto if_asap = params.get_input<int>("If asap");

    // Validate input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Need Geometry Input.");
    }

    // Convert to OpenMesh format
    auto ref_mesh = operand_to_openmesh(&ref_input);
    auto halfedge_mesh = operand_to_openmesh(&input);

    // Run ARAP solver
    ARAP arap_solver(halfedge_mesh, ref_mesh, static_cast<bool>(if_asap));
    arap_solver.compute(T);

    // Get results
    auto geometry = openmesh_to_operand(arap_solver.get_ans_mesh().get());

    // Set outputs
    params.set_output("OutputUV", arap_solver.get_ans_gfvec());
    params.set_output("Output", std::move(*geometry));
}

// Node UI declaration
NODE_DECLARATION_UI(arap);

NODE_DEF_CLOSE_SCOPE
