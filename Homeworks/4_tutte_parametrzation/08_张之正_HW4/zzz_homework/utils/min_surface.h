#pragma once

#include "GCore/Components/MeshOperand.h"
#include "GCore/util_openmesh_bind.h"
#include <Eigen/Sparse>
#include <memory>

void set_min_surface(float(*weight)(const OpenMesh::Vec3f&, const OpenMesh::Vec3f&, const OpenMesh::Vec3f&),
                  std::shared_ptr<USTC_CG::PolyMesh> reference_mesh,
                  std::shared_ptr<USTC_CG::PolyMesh> mesh);

float uniform_weights(const OpenMesh::Vec3f&, const OpenMesh::Vec3f&, const OpenMesh::Vec3f&);

float shape_preserving_weights(const OpenMesh::Vec3f& vec0, const OpenMesh::Vec3f& vecl, const OpenMesh::Vec3f& vecr);

float cotangent_weights(const OpenMesh::Vec3f& vec0, const OpenMesh::Vec3f& vecl, const OpenMesh::Vec3f& vecr);

