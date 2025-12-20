#pragma once
#include <vector>
#include <utility> // for std::pair
#include <imgui.h> // for ImVec2

namespace USTC_CG
{
class Warper
{
   public:
    virtual ~Warper() = default;
    
    //  (x,y) to (x', y')
    virtual std::pair<float, float> warp(float x, float y) = 0;
    
    virtual void set_control_points(
        const std::vector<std::pair<ImVec2, ImVec2>>& control_points) = 0;
};
} // namespace USTC_CG