// File: RBF_warper.cpp
#include "RBF_warper.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include "../../../../include/common/image.h"

namespace USTC_CG
{
    RBFWarper::RBFWarper(
        const std::vector<std::pair<ImVec2, ImVec2>>& control_points,
        int width, int height
    ) : width_(width), height_(height)
    {
        set_control_points(control_points);
    }

void RBFWarper::set_control_points(const std::vector<std::pair<ImVec2, ImVec2>>& control_points)
{
    control_points_.clear();
    for (const auto& pair : control_points) {
        ControlPoint cp;
        cp.px = pair.second.x; // Original image point (start_point)
        cp.py = pair.second.y;
        cp.qx = pair.first.x;  // Target point (end_point)
        cp.qy = pair.first.y;
        control_points_.push_back(cp);
    }
    compute_parameters();
}

void RBFWarper::compute_parameters()
{
    const int n = control_points_.size();
    if (n == 0) return;

    // Step 1: Calculate radius for each control point: r_i = min_j ||p_i - p_j||
    for (auto& cp : control_points_) {
        float min_dist = std::numeric_limits<float>::max();
        if (n == 1) {
            // For a single control point, use a default radius value
            min_dist = std::min(width_, height_) * 0.1f; 
        } else {
            for (const auto& other : control_points_) {
                if (&cp == &other) continue;
                float dx = cp.px - other.px;
                float dy = cp.py - other.py;
                float dist = std::hypot(dx, dy);
                if (dist < min_dist) min_dist = dist;
            }
        }
        cp.r = (min_dist < 1e-6f) ? 1.0f : min_dist; // Prevent division by zero
    }

    // Step 2: Solve the linear system of equations
    const int m = n + 3; // Matrix dimension
    Eigen::MatrixXf G = Eigen::MatrixXf::Zero(m, m);
    Eigen::VectorXf rhs_x(m), rhs_y(m);

    // Fill the matrix G with radial basis function values
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            float dx = control_points_[i].px - control_points_[j].px;
            float dy = control_points_[i].py - control_points_[j].py;
            float dist = std::hypot(dx, dy);
            G(i, j) = rbf_function(dist, control_points_[j].r);
        }
        // Add polynomial terms [p_i^T, 1]
        G(i, n)     = control_points_[i].px;
        G(i, n + 1) = control_points_[i].py;
        G(i, n + 2) = 1.0f;

        // Right-hand side: (q_i - p_i) for the displacement
        rhs_x(i) = control_points_[i].qx - control_points_[i].px;
        rhs_y(i) = control_points_[i].qy - control_points_[i].py;
    }

    // Add polynomial constraints: sum of weights = 0
    for (int j = 0; j < n; ++j) {
        G(n, j)     = control_points_[j].px;
        G(n + 1, j) = control_points_[j].py;
        G(n + 2, j) = 1.0f;
    }
    // Complete the matrix with zeros in the lower right
    G(n, n) = G(n, n + 1) = G(n, n + 2) = 0.0f;
    G(n + 1, n) = G(n + 1, n + 1) = G(n + 1, n + 2) = 0.0f;
    G(n + 2, n) = G(n + 2, n + 1) = G(n + 2, n + 2) = 0.0f;
    rhs_x(n) = rhs_x(n + 1) = rhs_x(n + 2) = 0.0f;
    rhs_y(n) = rhs_y(n + 1) = rhs_y(n + 2) = 0.0f;

    // Solve the linear system
    Eigen::ColPivHouseholderQR<Eigen::MatrixXf> solver(G);
    alpha_x_ = solver.solve(rhs_x);
    alpha_y_ = solver.solve(rhs_y);
    
    // Store the coefficients safely
    coefficients_x_.resize(n + 3);
    coefficients_y_.resize(n + 3);
    for(int i = 0; i < n + 3; ++i) {
        coefficients_x_[i] = alpha_x_(i);
        coefficients_y_[i] = alpha_y_(i);
    }
}

// Radial basis function: inverse multiquadric
float RBFWarper::rbf_function(float distance, float radius) {
    return 1.0f / std::sqrt(distance * distance + radius * radius);
}

std::pair<float, float> RBFWarper::warp(float x, float y)
{
    if (control_points_.empty()) return {x, y};

    // Initialize with the original coordinates
    float src_x = x;
    float src_y = y;
    
    // Calculate radial basis function values
    float fx = 0.0f, fy = 0.0f;
    for (size_t i = 0; i < control_points_.size(); ++i) {
        float dx = x - control_points_[i].px;
        float dy = y - control_points_[i].py;
        float dist = std::hypot(dx, dy);
        float g = rbf_function(dist, control_points_[i].r);
        fx += coefficients_x_[i] * g;
        fy += coefficients_y_[i] * g;
    }

    const int n = control_points_.size();

    // Add affine transformation component
    fx += coefficients_x_[n] * x + coefficients_x_[n + 1] * y + coefficients_x_[n + 2];
    fy += coefficients_y_[n] * x + coefficients_y_[n + 1] * y + coefficients_y_[n + 2];

    // Calculate source coordinates (x + displacement -> source)
    src_x = x + fx;
    src_y = y + fy;
    
    // Ensure coordinates stay within image boundaries
    src_x = std::clamp(src_x, 0.0f, static_cast<float>(width_ - 1));
    src_y = std::clamp(src_y, 0.0f, static_cast<float>(height_ - 1));

    return {src_x, src_y};
}

void RBFWarper::set_image_dimensions(int width, int height) {
    width_ = width;
    height_ = height;
}

} // namespace USTC_CG