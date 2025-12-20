#pragma once
#include "warper.h" 
#include <vector>
#include <utility> // for std::pair
#include <imgui.h> // for ImVec2

namespace USTC_CG
{
class IDWWarper : public Warper 
{
public:
    IDWWarper() = default;
    explicit IDWWarper(const std::vector<std::pair<ImVec2, ImVec2>>& control_points, float mu = 2.0f);
    
    void set_control_points(const std::vector<std::pair<ImVec2, ImVec2>>& control_points) override;
    std::pair<float, float> warp(float x, float y) override;

private:
    struct ControlPoint {
        float px, py; // Original image coordinates p_i
        float qx, qy; // Target coordinates q_i
        float a, b, c, d; // Local transformation matrix T_i = [[a, b], [c, d]]
    };
    
    void compute_transformations();
    std::vector<ControlPoint> control_points_;
    float mu_ = 2.0f; // IDW parameter
};
} // namespace USTC_CG