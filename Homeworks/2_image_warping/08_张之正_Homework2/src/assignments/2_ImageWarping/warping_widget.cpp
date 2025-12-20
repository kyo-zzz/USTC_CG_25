#include "warping_widget.h"
#include "warper/warper.h"
#include "warper/IDW_warper.h" 
#include "warper/RBF_warper.h" 
#include <cmath>
#include <iostream>
#include <algorithm> // for std::clamp

namespace USTC_CG
{
using uchar = unsigned char;

WarpingWidget::WarpingWidget(const std::string& label, const std::string& filename)
    : ImageWidget(label, filename)
{
    if (data_)
        back_up_ = std::make_shared<Image>(*data_);
}

void WarpingWidget::draw()
{
    // Draw the image
    ImageWidget::draw();
    // Draw the canvas
    if (flag_enable_selecting_points_)
        select_points();
}

void WarpingWidget::invert()
{
    for (int i = 0; i < data_->width(); ++i)
    {
        for (int j = 0; j < data_->height(); ++j)
        {
            const auto color = data_->get_pixel(i, j);
            data_->set_pixel(
                i,
                j,
                { static_cast<uchar>(255 - color[0]),
                  static_cast<uchar>(255 - color[1]),
                  static_cast<uchar>(255 - color[2]) });
        }
    }
    // After change the image, we should reload the image data to the renderer
    update();
}

void WarpingWidget::mirror(bool is_horizontal, bool is_vertical)
{
    Image image_tmp(*data_);
    int width = data_->width();
    int height = data_->height();

    if (is_horizontal)
    {
        if (is_vertical)
        {
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    data_->set_pixel(
                        i,
                        j,
                        image_tmp.get_pixel(width - 1 - i, height - 1 - j));
                }
            }
        }
        else
        {
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    data_->set_pixel(
                        i, j, image_tmp.get_pixel(width - 1 - i, j));
                }
            }
        }
    }
    else
    {
        if (is_vertical)
        {
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    data_->set_pixel(
                        i, j, image_tmp.get_pixel(i, height - 1 - j));
                }
            }
        }
    }

    // After change the image, we should reload the image data to the renderer
    update();
}

void WarpingWidget::gray_scale()
{
    for (int i = 0; i < data_->width(); ++i)
    {
        for (int j = 0; j < data_->height(); ++j)
        {
            const auto color = data_->get_pixel(i, j);
            uchar gray_value = (color[0] + color[1] + color[2]) / 3;
            data_->set_pixel(i, j, { gray_value, gray_value, gray_value });
        }
    }
    // After change the image, we should reload the image data to the renderer
    update();
}

void WarpingWidget::warping()
{
    // Create a warped image with the same dimensions as the original
    int width = data_->width();
    int height = data_->height();
    Image warped_image(width, height, data_->channels());
    
    // Initialize warped image to black
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            warped_image.set_pixel(x, y, {0, 0, 0});
        }
    }

    switch (warping_type_) {
        case kFisheye: {
            // Forward mapping: from source to destination
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    auto [new_x, new_y] = fisheye_warping(x, y, width, height);
                    
                    // Ensure the new coordinates are within bounds
                    if (new_x >= 0 && new_x < width && new_y >= 0 && new_y < height) {
                        auto color = data_->get_pixel(x, y);
                        warped_image.set_pixel(new_x, new_y, color);
                    }
                }
            }
            
            // Fill in any gaps caused by discrete mapping
            break;
        }
        
        case kIDW: {
            std::vector<std::pair<ImVec2, ImVec2>> control_points;
            for (size_t i = 0; i < start_points_.size(); ++i) {
                // Control points mapping: from end points to start points
                control_points.emplace_back(end_points_[i], start_points_[i]);
            }
            
            // Initialize the IDW warper with control points and appropriate parameters
            IDWWarper warper(control_points, 2.0f);
            
            // Backward mapping: for each pixel in the destination, find its source
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    // Find corresponding source coordinates
                    auto [src_x, src_y] = warper.warp(static_cast<float>(x), static_cast<float>(y));
                    
                    // Use bilinear interpolation to get the color
                    auto color = interpolate_bilinear(src_x, src_y);
                    warped_image.set_pixel(x, y, color);
                }
            }
            break;
        }
        
        case kRBF: {
            std::vector<std::pair<ImVec2, ImVec2>> control_points;
            for (size_t i = 0; i < start_points_.size(); ++i) {
                control_points.emplace_back( start_points_[i],end_points_[i]);
            }
            
            // Initialize the RBF warper with control points and image dimensions
            RBFWarper warper(control_points, width, height);
            
            // Backward mapping: for each pixel in the destination, find its source
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    auto [src_x, src_y] = warper.warp(static_cast<float>(x), static_cast<float>(y));
                    
                    // Use bilinear interpolation to get the color
                    auto color = interpolate_bilinear(src_x, src_y);
                    warped_image.set_pixel(x, y, color);
                }
            }
            break;
        }
        
        default: 
            break;
    }

    // Update the image data with the warped result
    *data_ = std::move(warped_image);
    update();
}


// Bilinear interpolation (with boundary checks)
std::vector<unsigned char> WarpingWidget::interpolate_bilinear(float x, float y)
{
    // Clamp coordinates to valid range
    x = std::clamp(x, 0.0f, static_cast<float>(data_->width() - 1));
    y = std::clamp(y, 0.0f, static_cast<float>(data_->height() - 1));

    int x0 = static_cast<int>(x);
    int y0 = static_cast<int>(y);
    int x1 = std::min(x0 + 1, data_->width() - 1);
    int y1 = std::min(y0 + 1, data_->height() - 1);
    float dx = x - x0;
    float dy = y - y0;

    auto c00 = data_->get_pixel(x0, y0);
    auto c01 = data_->get_pixel(x0, y1);
    auto c10 = data_->get_pixel(x1, y0);
    auto c11 = data_->get_pixel(x1, y1);

    std::vector<unsigned char> result(data_->channels());
    for (int ch = 0; ch < data_->channels(); ++ch) {
        float val = 
            c00[ch] * (1 - dx) * (1 - dy) +
            c10[ch] * dx * (1 - dy) +
            c01[ch] * (1 - dx) * dy +
            c11[ch] * dx * dy;
        result[ch] = static_cast<unsigned char>(std::clamp(val, 0.0f, 255.0f));
    }
    return result;
}

void WarpingWidget::restore()
{
    *data_ = *back_up_;
    update();
}

void WarpingWidget::set_default()
{
    warping_type_ = kDefault;
}

void WarpingWidget::set_fisheye()
{
    warping_type_ = kFisheye;
}

void WarpingWidget::set_IDW()
{
    warping_type_ = kIDW;
}

void WarpingWidget::set_RBF()
{
    warping_type_ = kRBF;
}

void WarpingWidget::enable_selecting(bool flag)
{
    flag_enable_selecting_points_ = flag;
}

void WarpingWidget::select_points()
{
    /// Invisible button over the canvas to capture mouse interactions.
    ImGui::SetCursorScreenPos(position_);
    ImGui::InvisibleButton(
        label_.c_str(),
        ImVec2(
            static_cast<float>(image_width_),
            static_cast<float>(image_height_)),
        ImGuiButtonFlags_MouseButtonLeft);
    // Record the current status of the invisible button
    bool is_hovered_ = ImGui::IsItemHovered();
    // Selections
    ImGuiIO& io = ImGui::GetIO();
    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        draw_status_ = true;
        start_ = end_ =
            ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
    }
    if (draw_status_)
    {
        end_ = ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            start_points_.push_back(start_);
            end_points_.push_back(end_);
            draw_status_ = false;
        }
    }
    // Visualization
    auto draw_list = ImGui::GetWindowDrawList();
    for (size_t i = 0; i < start_points_.size(); ++i)
    {
        ImVec2 s(
            start_points_[i].x + position_.x, start_points_[i].y + position_.y);
        ImVec2 e(
            end_points_[i].x + position_.x, end_points_[i].y + position_.y);
        draw_list->AddLine(s, e, IM_COL32(255, 0, 0, 255), 2.0f);
        draw_list->AddCircleFilled(s, 4.0f, IM_COL32(0, 0, 255, 255));
        draw_list->AddCircleFilled(e, 4.0f, IM_COL32(0, 255, 0, 255));
    }
    if (draw_status_)
    {
        ImVec2 s(start_.x + position_.x, start_.y + position_.y);
        ImVec2 e(end_.x + position_.x, end_.y + position_.y);
        draw_list->AddLine(s, e, IM_COL32(255, 0, 0, 255), 2.0f);
        draw_list->AddCircleFilled(s, 4.0f, IM_COL32(0, 0, 255, 255));
    }
}

void WarpingWidget::init_selections()
{
    start_points_.clear();
    end_points_.clear();
}

std::pair<int, int>
WarpingWidget::fisheye_warping(int x, int y, int width, int height)
{
    float center_x = width / 2.0f;
    float center_y = height / 2.0f;
    float dx = x - center_x;
    float dy = y - center_y;
    float distance = std::sqrt(dx * dx + dy * dy);
    
    // Maximum radius (from center to corner)
    float max_radius = std::sqrt(center_x * center_x + center_y * center_y);
    
    // Normalize the distance to [0, 1] range
    float normalized_distance = distance / max_radius;
    
    // Apply non-linear transformation to create fisheye effect
    // Use a function that ensures the boundary points remain fixed
    float distortion_factor = 0.5f; // Adjust for stronger/weaker effect
    float new_normalized_distance = std::pow(normalized_distance, distortion_factor);
    
    // Scale back to original range
    float new_distance = new_normalized_distance * max_radius;
    
    // Calculate new position
    int new_x, new_y;
    if (distance < 1e-6) { // Handle center point
        new_x = static_cast<int>(center_x);
        new_y = static_cast<int>(center_y);
    } else {
        float ratio = new_distance / distance;
        new_x = static_cast<int>(center_x + dx * ratio);
        new_y = static_cast<int>(center_y + dy * ratio);
    }
    
    // Make sure coordinates stay within the original image boundaries
    new_x = std::clamp(new_x, 0, width - 1);
    new_y = std::clamp(new_y, 0, height - 1);
    
    return {new_x, new_y};
}

}  // namespace USTC_CG