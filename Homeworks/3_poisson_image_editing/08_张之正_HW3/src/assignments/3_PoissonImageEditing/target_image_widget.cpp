#include "target_image_widget.h"
#include <chrono>
#include <cmath>
namespace USTC_CG
{
using uchar = unsigned char;

TargetImageWidget::TargetImageWidget(
    const std::string& label,
    const std::string& filename)
    : ImageWidget(label, filename)
{
    if (data_)
        back_up_ = std::make_shared<Image>(*data_);
}

void TargetImageWidget::draw()
{
    // Draw the image
    ImageWidget::draw();
    // Invisible button for interactions
    ImGui::SetCursorScreenPos(position_);
    ImGui::InvisibleButton(
        label_.c_str(),
        ImVec2(
            static_cast<float>(image_width_),
            static_cast<float>(image_height_)),
        ImGuiButtonFlags_MouseButtonLeft);
    bool is_hovered_ = ImGui::IsItemHovered();
    // When the mouse is clicked or moving, we would adapt clone function to
    // copy the selected region to the target.

    if (is_hovered_ && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        mouse_click_event();
    }
    mouse_move_event();
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
    {
        mouse_release_event();
    }
}

void TargetImageWidget::set_source(std::shared_ptr<SourceImageWidget> source)
{
    source_image_ = source;
}

void TargetImageWidget::set_realtime(bool flag)
{
    flag_realtime_updating = flag;
}

void TargetImageWidget::restore()
{
    *data_ = *back_up_;
    update();
}

void TargetImageWidget::set_paste()
{
    clone_type_ = kPaste;
}

void TargetImageWidget::set_seamless()
{
    clone_type_ = kSeamless;
}

void TargetImageWidget::clone()
{
    // The implementation of different types of cloning
    // HW3_TODO:
    // 1. In this function, you should at least implement the "seamless"
    // cloning labeled by `clone_type_ ==kSeamless`.
    //
    // 2. It is required to improve the efficiency of your seamless cloning to
    // achieve real-time editing. (Use decomposition of sparse matrix before
    // solve the linear system). The real-time updating (update when the mouse
    // is moving) is only available when the checkerboard is selected.
    if (data_ == nullptr || source_image_ == nullptr ||
        source_image_->get_region_mask() == nullptr)
        return;
    // The selected region in the source image, this would be a binary mask.
    // The **size** of the mask should be the same as the source image.
    // The **value** of the mask should be 0 or 255: 0 for the background and
    // 255 for the selected region.
    std::shared_ptr<Image> mask = source_image_->get_region_mask();
    int Soffset_x = static_cast<int>(source_image_->get_position().x);
    int Soffset_y = static_cast<int>(source_image_->get_position().y);
    int Toffset_x = static_cast<int>(mouse_position_.x);
    int Toffset_y = static_cast<int>(mouse_position_.y);

    switch (clone_type_)
    {
        case USTC_CG::TargetImageWidget::kDefault: break;
        case USTC_CG::TargetImageWidget::kPaste:
        {
            restore();  // Restore target image to initial state

            // Create Paste algorithm instance (using CloneMethod base class pointer)
            clone_method = std::make_shared<USTC_CG::Paste>(  
                source_image_->get_data(),
                data_,
                mask);

            // Execute cloning algorithm and get result
            std::shared_ptr<USTC_CG::Image> result = clone_method->solve();

            // Apply result to target image
            apply_clone(result, mask);
            break;
        }
        case USTC_CG::TargetImageWidget::kSeamless:
        {
            restore();  // Restore target image to initial state

            // Create SeamlessCloner instance (using CloneMethod base class pointer)
            clone_method = std::make_shared<USTC_CG::SeamlessCloner>(
                source_image_->get_data(),  // source image
                data_,                      // target image
                mask,                       // mask image
                Soffset_x,                  // source x offset
                Soffset_y,                  // source y offset
                Toffset_x,                  // target x offset
                Toffset_y                   // target y offset
            );

            // Execute cloning algorithm and get result
            std::shared_ptr<USTC_CG::Image> result = clone_method->solve();

            // Apply result to target image
            apply_clone(result, mask);
            break;
        }
    }

    update();
}


void TargetImageWidget::mouse_click_event()
{
    edit_status_ = true;
    mouse_position_ = mouse_pos_in_canvas();
    clone();
}

void TargetImageWidget::mouse_move_event()
{
    if (edit_status_)
    {
        mouse_position_ = mouse_pos_in_canvas();
        if (flag_realtime_updating)
            clone();
    }
}

void TargetImageWidget::mouse_release_event()
{
    if (edit_status_)
    {
        edit_status_ = false;
    }
}

void TargetImageWidget::apply_clone(const std::shared_ptr<Image>& result, const std::shared_ptr<Image>& mask)
{
    for (int x = 0; x < mask->width(); ++x)
    {
        for (int y = 0; y < mask->height(); ++y)
        {
            int tar_x =static_cast<int>(mouse_position_.x) + x -static_cast<int>(source_image_->get_position().x);
            int tar_y =static_cast<int>(mouse_position_.y) + y -static_cast<int>(source_image_->get_position().y);
            if (0 <= tar_x && tar_x < image_width_ && 0 <= tar_y &&
                tar_y < image_height_ && mask->get_pixel(x, y)[0] > 0)
            {
                data_->set_pixel(tar_x,tar_y,result->get_pixel(x, y));
            }
        }
    }
}

ImVec2 TargetImageWidget::mouse_pos_in_canvas() const
{
    ImGuiIO& io = ImGui::GetIO();
    return ImVec2(io.MousePos.x - position_.x, io.MousePos.y - position_.y);
}

}  // namespace USTC_CG