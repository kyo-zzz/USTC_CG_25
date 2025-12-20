// File: RBF_warper.h
#pragma once
#include "warper.h"
#include <Eigen/Dense>
#include <vector>
#include <imgui.h>

namespace USTC_CG
{
class RBFWarper : public Warper
{
public:
    RBFWarper() = default;
    explicit RBFWarper(
        const std::vector<std::pair<ImVec2, ImVec2>>& control_points,
        int width = 0,  // Original image width
        int height = 0  // Original image height
    );

    void set_control_points(const std::vector<std::pair<ImVec2, ImVec2>>& control_points) override;
    std::pair<float, float> warp(float x, float y) override;
    
    // Set image dimensions (useful when constructed with default constructor)
    void set_image_dimensions(int width, int height);

private:
    int width_ = 0;   // Original image width
    int height_ = 0;  // Original image height
    
    // Control point data structure
    struct ControlPoint {
        float px, py; // Original image point p_i
        float qx, qy; // Target point q_i
        float r;      // Radius parameter r_i = min_j ||p_i - p_j||
    };

    // Calculate RBF parameters
    void compute_parameters();
    
    // Radial basis function implementation
    float rbf_function(float distance, float radius);

    std::vector<ControlPoint> control_points_; // List of control points
    Eigen::VectorXf alpha_x_, alpha_y_;       // RBF coefficients (Eigen type)
    std::vector<float> coefficients_x_, coefficients_y_; // RBF coefficients (std::vector type)
};

} // namespace USTC_CG