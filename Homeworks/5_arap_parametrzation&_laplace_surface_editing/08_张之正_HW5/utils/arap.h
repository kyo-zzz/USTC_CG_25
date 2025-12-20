#pragma once

#include "GCore/Components/MeshOperand.h"
#include "GCore/util_openmesh_bind.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <vector>
#include <time.h>

class ARAP {
public:
  ARAP(std::shared_ptr<USTC_CG::PolyMesh> ref,
     std::shared_ptr<USTC_CG::PolyMesh> mesh,
     bool asap);

  void set_origin(std::shared_ptr<USTC_CG::PolyMesh> mesh);
  void set_ref(std::shared_ptr<USTC_CG::PolyMesh> mesh);
  void compute(int iter);
  std::shared_ptr<USTC_CG::PolyMesh> get_ans_mesh();
  pxr::VtArray<pxr::GfVec2f> get_ans_gfvec();

private:
  std::shared_ptr<USTC_CG::PolyMesh> ref_mesh_;
  std::shared_ptr<USTC_CG::PolyMesh> halfedge_mesh_;

  std::map<int, int> index_to_i_;
  std::vector<int> i_to_index_;

  Eigen::SparseMatrix<double> A_;
  Eigen::VectorXd B_[2];
  Eigen::VectorXd solution_[2];
  Eigen::SparseLU<Eigen::SparseMatrix<double>> solver_;
  std::vector<Eigen::Triplet<double>> triplet_list_;

  std::map<int, std::vector<double>> next_cot_;
  std::map<int, std::vector<double>> last_cot_;
  std::map<int, std::vector<Eigen::Matrix2d>> projections_;
  std::map<int, std::vector<Eigen::Matrix2d>> current_triangle_;
  std::map<int, std::vector<Eigen::Matrix2d>> L_t_;
  std::map<int, Eigen::Vector2d> T_dx_;

  pxr::VtArray<pxr::GfVec2f> uv_result_;
  bool if_asap_ = false;

  Eigen::Matrix2d SpinToSurf(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
  Eigen::Matrix2d SVD(const Eigen::Matrix2d& target);
};
