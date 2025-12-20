#include "IDW_warper.h"
#include <cmath>
#include <algorithm> // for std::hypot, std::pow

namespace USTC_CG {
    // Constructs an IDWWarper object with given control points and mu value.
    IDWWarper::IDWWarper(
        const std::vector<std::pair<ImVec2, ImVec2>>& control_points, float mu)
        : mu_(mu)
    {
        set_control_points(control_points);
    }

    // Sets the control points for the warper and computes the transformations.
    void IDWWarper::set_control_points(
        const std::vector<std::pair<ImVec2, ImVec2>>& control_points)
    {
        control_points_.clear();
        for (const auto& pair : control_points) {
            ControlPoint cp;
            cp.px = pair.first.x;
            cp.py = pair.first.y;
            cp.qx = pair.second.x;
            cp.qy = pair.second.y;
            control_points_.push_back(cp);
        }
        compute_transformations();
    }

    // Computes the transformation matrices for each control point.
    void IDWWarper::compute_transformations()
    {
        for (auto& cp : control_points_) {
            float A11 = 0, A12 = 0, A21 = 0, A22 = 0;
            float B11 = 0, B12 = 0, B21 = 0, B22 = 0;

            for (const auto& other : control_points_) {
                if (&cp == &other) continue;

                float dx = other.px - cp.px;
                float dy = other.py - cp.py;
                float dist = std::hypot(dx, dy);
                float sigma = 1.0f / std::pow(dist, mu_);

                A11 += sigma * dx * dx;
                A12 += sigma * dx * dy;
                A21 += sigma * dy * dx;
                A22 += sigma * dy * dy;

                float dqx = other.qx - cp.qx;
                float dqy = other.qy - cp.qy;
                B11 += sigma * dqx * dx;
                B12 += sigma * dqx * dy;
                B21 += sigma * dqy * dx;
                B22 += sigma * dqy * dy;
            }

            float det = A11 * A22 - A12 * A21;
            if (std::abs(det) < 1e-6) {
                cp.a = cp.d = 1.0f;
                cp.b = cp.c = 0.0f;
                continue;
            }

            float inv_det = 1.0f / det;
            float invA11 =  A22 * inv_det;
            float invA12 = -A12 * inv_det;
            float invA21 = -A21 * inv_det;
            float invA22 =  A11 * inv_det;

            cp.a = B11 * invA11 + B12 * invA21;
            cp.b = B11 * invA12 + B12 * invA22;
            cp.c = B21 * invA11 + B22 * invA21;
            cp.d = B21 * invA12 + B22 * invA22;
        }
    }

    // Warps a given point (x, y) using the IDW algorithm.
    std::pair<float, float> IDWWarper::warp(float x, float y) 
    {
        float sum_weights = 0.0f;
        float result_x = 0.0f, result_y = 0.0f;

        for (const auto& cp : control_points_) {
            float dx = x - cp.px;
            float dy = y - cp.py;
            float dist = std::hypot(dx, dy);
            
            if (dist < 1e-6) {
                return {cp.qx, cp.qy};
            }

            float sigma = 1.0f / std::pow(dist, mu_);
            sum_weights += sigma;

            float tx = cp.a * dx + cp.b * dy;
            float ty = cp.c * dx + cp.d * dy;
            float fx = cp.qx + tx;
            float fy = cp.qy + ty;

            result_x += sigma * fx;
            result_y += sigma * fy;
        }

        if (sum_weights > 0) {
            result_x /= sum_weights;
            result_y /= sum_weights;
        }
        return {result_x, result_y};
    }
} // namespace USTC_CG