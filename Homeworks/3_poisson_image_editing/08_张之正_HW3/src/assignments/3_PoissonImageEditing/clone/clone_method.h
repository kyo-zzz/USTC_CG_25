#pragma once

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

#include "common/image_widget.h"

namespace USTC_CG
{

class CloneMethod
{
   public:
    CloneMethod(
        std::shared_ptr<Image> src_img,
        std::shared_ptr<Image> tar_img,
        std::shared_ptr<Image> src_selected_mask)
        : source_image_(src_img),
          target_image_(tar_img),
          mask_image_(src_selected_mask)
    {
    }

    virtual ~CloneMethod() = default;

    // 纯虚函数：子类需实现具体的克隆算法
    virtual std::shared_ptr<Image> solve() = 0;

    // 访问器函数
    std::shared_ptr<Image> source_image() const
    {
        return source_image_;
    }
    std::shared_ptr<Image> target_image() const
    {
        return target_image_;
    }
    std::shared_ptr<Image> mask_image() const
    {
        return mask_image_;
    }

   private:
    std::shared_ptr<Image> source_image_;  // 源图像（待克隆内容）
    std::shared_ptr<Image> target_image_;  // 目标图像（克隆到的背景）
    std::shared_ptr<Image> mask_image_;    // 掩码（标记克隆区域）
};

}  // namespace USTC_CG