// seamless_cloner.h (Variable Optimization)
#pragma once
#include "clone_method.h"
#include <Eigen/Sparse>

namespace USTC_CG
{
class SeamlessCloner : public CloneMethod {
 public:
    // Constructor parameter renaming
    SeamlessCloner(
        std::shared_ptr<Image> source_image,
        std::shared_ptr<Image> target_image,
        std::shared_ptr<Image> selection_mask,
        int source_offset_x, int source_offset_y,
        int target_offset_x, int target_offset_y)
        : CloneMethod(source_image, target_image, selection_mask),
            source_offset_x_(source_offset_x),
            source_offset_y_(source_offset_y),
            target_offset_x_(target_offset_x),
            target_offset_y_(target_offset_y) {
        calculateMaskDimensions();
        initializeCoefficientMatrix();
        initializeBoundaryVectors();
    }

    std::shared_ptr<Image> solve() override;

    // Member function renaming
    void calculateMaskDimensions();
    bool isPixelInMask(int x, int y);

    // Getter renaming
    int source_offset_x() const { return source_offset_x_; }
    int source_offset_y() const { return source_offset_y_; }

 private:
    void initializeCoefficientMatrix();
    void initializeBoundaryVectors();
    std::vector<unsigned char> getSourcePixel(int x, int y);

    // Variable renaming
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> linear_solver_;
    Eigen::VectorXd boundary_red_, boundary_green_, boundary_blue_;
    
    int source_offset_x_, source_offset_y_;
    int target_offset_x_, target_offset_y_;
    int mask_width_ = 0, mask_height_ = 0;
    bool is_matrix_initialized_ = false;
};
}  // namespace USTC_CG