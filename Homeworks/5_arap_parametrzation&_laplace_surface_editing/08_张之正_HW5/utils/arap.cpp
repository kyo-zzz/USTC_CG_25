#include "arap.h"

ARAP::ARAP(std::shared_ptr<USTC_CG::PolyMesh> mesh, std::shared_ptr<USTC_CG::PolyMesh> ref, bool if_asap)
    : halfedge_mesh(mesh), ref_mesh(ref), if_asap_(if_asap) {}

void ARAP::set_origin(std::shared_ptr<USTC_CG::PolyMesh> mesh) {
    halfedge_mesh = mesh;
}

void ARAP::set_ref(std::shared_ptr<USTC_CG::PolyMesh> mesh) {
    ref_mesh = mesh;
}

Eigen::Matrix2d ARAP::SpinToSurf(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    Eigen::Vector3d n0_normalized = (a.cross(b)).normalized();
    Eigen::Vector3d n1_normalized(0, 0, 1.0);
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(n0_normalized, n1_normalized);
    Eigen::Matrix3d T = q.toRotationMatrix();
    Eigen::Vector3d ap = T * a;
    Eigen::Vector3d bp = T * b;
    Eigen::Matrix2d final_mat;
    final_mat << ap(0), bp(0),
                 ap(1), bp(1);
    return final_mat;
}

Eigen::Matrix2d ARAP::SVD(Eigen::Matrix2d target) {
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(target, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Vector2d S = svd.singularValues();
    Eigen::Matrix2d V = svd.matrixV();
    if (if_asap_)
        return U * Eigen::Matrix2d::Identity() * 2 * std::abs(S.mean()) * V.transpose();
    return U * Eigen::Matrix2d::Identity() * V.transpose();
}

void ARAP::compute(int times) {
    i_to_index.clear();
    index_to_i.clear();
    tripletList.clear();

    // Build index mapping
    for (const auto& vertex_handle : halfedge_mesh->vertices()) {
        index_to_i[vertex_handle.idx()] = i_to_index.size();
        i_to_index.push_back(vertex_handle.idx());
    }

    // Construct Laplacian matrix
    for (const auto& vertex_handle : ref_mesh->vertices()) {
        const auto& position = ref_mesh->point(vertex_handle);
        double sum_w = 0;
        int idx = index_to_i[vertex_handle.idx()];
        if (idx == 0) {
            tripletList.emplace_back(idx, idx, 1);
            continue;
        }
        int num = 0;
        if (vertex_handle.is_boundary()) {
            for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges()) ++num;
        }
        int t_handle = 0;
        std::vector<double> current_next_cot, current_last_cot;
        for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges()) {
            const auto& v0 = halfedge_handle.to();
            const auto& vl = halfedge_handle.opp().prev().prev().to();
            const auto& vr = halfedge_handle.prev().opp().to();
            const auto& vec0 = ref_mesh->point(v0) - position;
            const auto& vecl = ref_mesh->point(vl) - position;
            const auto& vecr = ref_mesh->point(vr) - position;
            current_next_cot.push_back((vecl - vec0).dot(vecl) / vecl.cross(vec0).length());
            current_last_cot.push_back((vecr - vec0).dot(vecr) / vecr.cross(vec0).length());
            if (num == 0 || t_handle > 0) {
                tripletList.emplace_back(idx, index_to_i[v0.idx()], -current_last_cot.back());
                sum_w += current_last_cot.back();
            } else {
                current_last_cot.back() = 0;
            }
            if (num == 0 || t_handle < num - 1) {
                tripletList.emplace_back(idx, index_to_i[v0.idx()], -current_next_cot.back());
                sum_w += current_next_cot.back();
            } else {
                current_next_cot.back() = 0;
            }
            ++t_handle;
        }
        tripletList.emplace_back(idx, idx, sum_w);
        next_cot[vertex_handle.idx()] = current_next_cot;
        last_cot[vertex_handle.idx()] = current_last_cot;
    }

    A_.resize(i_to_index.size(), i_to_index.size());
    A_.setFromTriplets(tripletList.begin(), tripletList.end());
    solver_.compute(A_);
    if (solver_.info() != Eigen::Success) {
        std::cerr << "failed" << std::endl;
    }

    // Precompute projections
    for (const auto& vertex_handle : ref_mesh->vertices()) {
        std::vector<Eigen::Matrix2d> proj;
        const auto& position = ref_mesh->point(vertex_handle);
        int num = 0;
        if (vertex_handle.is_boundary()) {
            for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges()) ++num;
        }
        int t_handle = 0;
        for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges()) {
            const auto& v0 = halfedge_handle.to();
            const auto& vl = halfedge_handle.opp().prev().prev().to();
            const auto& vec0 = ref_mesh->point(v0) - position;
            const auto& vecl = ref_mesh->point(vl) - position;
            Eigen::Vector3d v0_E(vec0[0], vec0[1], vec0[2]);
            Eigen::Vector3d vl_E(vecl[0], vecl[1], vecl[2]);
            proj.push_back(SpinToSurf(v0_E, vl_E));
            if (t_handle == num - 1) {
                proj.back().setZero();
            }
            ++t_handle;
        }
        projections[vertex_handle.idx()] = proj;
    }

    int t_max = times;
    for (int t = 1; t <= t_max; ++t) {
        std::cout << "Current iteration number: " << t << '/' << t_max << std::endl;
        for (int j = 0; j < 2; ++j) {
            B_[j] = Eigen::VectorXd::Zero(i_to_index.size());
        }
        for (const auto& vertex_handle : halfedge_mesh->vertices()) {
            std::vector<Eigen::Matrix2d> proj, L;
            const auto& position = halfedge_mesh->point(vertex_handle);
            for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges()) {
                const auto& v0 = halfedge_handle.to();
                const auto& vl = halfedge_handle.opp().prev().prev().to();
                const auto& vec0 = halfedge_mesh->point(v0) - position;
                const auto& vecl = halfedge_mesh->point(vl) - position;
                Eigen::Matrix2d current_shape;
                current_shape << vec0[0], vecl[0],
                                 vec0[1], vecl[1];
                L.push_back(SVD(current_shape * projections[vertex_handle.idx()][proj.size()].inverse()));
                proj.push_back(current_shape);
            }
            L_t[vertex_handle.idx()] = L;
            current_triangle[vertex_handle.idx()] = proj;
        }
        for (const auto& vertex_handle : halfedge_mesh->vertices()) {
            int i = 0;
            Eigen::Vector2d delta_x = Eigen::Vector2d::Zero();
            int idx = index_to_i[vertex_handle.idx()];
            if (idx == 0) {
                B_[0](idx) = 0;
                B_[1](idx) = 0;
                continue;
            }
            int num = last_cot[vertex_handle.idx()].size();
            for (const auto& halfedge_handle : vertex_handle.outgoing_halfedges()) {
                delta_x -= next_cot[vertex_handle.idx()][i] * L_t[vertex_handle.idx()][i] * projections[vertex_handle.idx()][i].col(0);
                delta_x -= last_cot[vertex_handle.idx()][(i + 1) % num] * L_t[vertex_handle.idx()][i] * projections[vertex_handle.idx()][i].col(1);
                ++i;
            }
            B_[0](idx) = delta_x(0);
            B_[1](idx) = delta_x(1);
        }
        for (int j = 0; j < 2; ++j) {
            solution_[j] = solver_.solve(B_[j]);
            for (const auto& vertex_handle : halfedge_mesh->vertices()) {
                halfedge_mesh->point(vertex_handle)[j] = solution_[j](index_to_i[vertex_handle.idx()]);
            }
        }
        float k = 0;
        if (if_asap_ || (t >= t_max - 2)) {
            for (const auto& vertex_handle : halfedge_mesh->vertices()) {
                k = std::max(k, std::abs(halfedge_mesh->point(vertex_handle)[0]));
            }
            for (const auto& vertex_handle : halfedge_mesh->vertices()) {
                halfedge_mesh->point(vertex_handle) /= k;
            }
        }
    }
}

pxr::VtArray<pxr::GfVec2f> ARAP::get_ans_gfvec() {
    for (const auto& vertex_handle : halfedge_mesh->vertices()) {
        pxr::GfVec2f vh_gfvec;
        vh_gfvec.Set((halfedge_mesh->point(vertex_handle))[0], (halfedge_mesh->point(vertex_handle))[1]);
        uv_result_.push_back(vh_gfvec);
    }
    return uv_result_;
}

std::shared_ptr<USTC_CG::PolyMesh> ARAP::get_ans_mesh() {
    return halfedge_mesh;
}
