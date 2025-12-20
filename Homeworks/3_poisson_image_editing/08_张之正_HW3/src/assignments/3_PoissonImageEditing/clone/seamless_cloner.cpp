#include "seamless_cloner.h"

namespace USTC_CG {

namespace {
    // Constants definition
    constexpr int NUM_CHANNELS = 3;
    constexpr unsigned char MASK_VALUE = 255;
}

bool SeamlessCloner::isPixelInMask(int x, int y) {
    if (!mask_image()) return true;

    int sourceX = x - target_offset_x_;
    int sourceY = y - target_offset_y_;

    return !(sourceX < 0 || sourceX >= mask_image()->width() || 
             sourceY < 0 || sourceY >= mask_image()->height());
}

std::vector<unsigned char> SeamlessCloner::getSourcePixel(int x, int y) {
    int realX = x + source_offset_x_;
    int realY = y + source_offset_y_;

    if (realX < 0 || realX >= source_image()->width() || 
        realY < 0 || realY >= source_image()->height()) {
        return std::vector<unsigned char>(NUM_CHANNELS, 0);
    }

    return source_image()->get_pixel(realX, realY);
}

void SeamlessCloner::calculateMaskDimensions() { 
    auto maskImage = mask_image();
    int minX = maskImage->width(), minY = maskImage->height();
    int maxX = 0, maxY = 0;

    // Find active mask area
    for (int y = 0; y < maskImage->height(); ++y) {
        for (int x = 0; x < maskImage->width(); ++x) {
            if (maskImage->get_pixel(x, y)[0] == MASK_VALUE) {
                minX = std::min(minX, x);
                minY = std::min(minY, y);
                maxX = std::max(maxX, x);
                maxY = std::max(maxY, y);
            }
        }
    }

    mask_width_ = (maxX >= minX) ? (maxX - minX + 1) : 0;
    mask_height_ = (maxY >= minY) ? (maxY - minY + 1) : 0;
}

void SeamlessCloner::initializeCoefficientMatrix() {
    Eigen::SparseMatrix<double> coeffMatrix(mask_width_ * mask_height_, 
                                          mask_width_ * mask_height_);
    std::vector<Eigen::Triplet<double>> coefficients;
    coefficients.reserve(5 * mask_width_ * mask_height_);

    // Build Laplacian operator coefficient matrix
    for (int y = 0; y < mask_height_; ++y) {
        for (int x = 0; x < mask_width_; ++x) {
            int idx = y * mask_width_ + x;
            coefficients.emplace_back(idx, idx, 4.0);
            
            if (x > 0)          coefficients.emplace_back(idx, idx - 1, -1.0);
            if (x < mask_width_ - 1)  coefficients.emplace_back(idx, idx + 1, -1.0);
            if (y > 0)          coefficients.emplace_back(idx, idx - mask_width_, -1.0);
            if (y < mask_height_ - 1) coefficients.emplace_back(idx, idx + mask_width_, -1.0);
        }
    }

    coeffMatrix.setFromTriplets(coefficients.begin(), coefficients.end());
    linear_solver_.compute(coeffMatrix);
    is_matrix_initialized_ = true;
}

void SeamlessCloner::initializeBoundaryVectors() {
    const int targetWidth = target_image()->width();
    const int targetHeight = target_image()->height();

    auto processChannel = [this, targetWidth, targetHeight](int channel) {
        Eigen::VectorXd boundaryVec(mask_width_ * mask_height_);

        for (int y = 0; y < mask_height_; ++y) {
            for (int x = 0; x < mask_width_; ++x) {
                int idx = y * mask_width_ + x;
                
                // Calculate guidance field
                boundaryVec(idx) = 4.0 * getSourcePixel(x, y)[channel] 
                                - getSourcePixel(x, y - 1)[channel] 
                                - getSourcePixel(x, y + 1)[channel] 
                                - getSourcePixel(x - 1, y)[channel] 
                                - getSourcePixel(x + 1, y)[channel];
                
                // Handle boundary conditions
                auto handleBoundary = [&](int offsetX, int offsetY) {
                    return target_image()->get_pixel(
                        std::clamp(x + offsetX + target_offset_x_, 0, targetWidth - 1),
                        std::clamp(y + offsetY + target_offset_y_, 0, targetHeight - 1))[channel];
                };

                if (x == 0) boundaryVec(idx) += handleBoundary(-1, 0);
                if (y == 0) boundaryVec(idx) += handleBoundary(0, -1);
                if (x == mask_width_ - 1)  boundaryVec(idx) += handleBoundary(1, 0);
                if (y == mask_height_ - 1) boundaryVec(idx) += handleBoundary(0, 1);
            }
        }

        return boundaryVec;
    };

    boundary_red_   = processChannel(0);
    boundary_green_ = processChannel(1);
    boundary_blue_  = processChannel(2);
}

std::shared_ptr<Image> SeamlessCloner::solve() {
    auto result = std::make_shared<Image>(*source_image());
    std::vector<unsigned char> solvedValues(mask_width_ * mask_height_ * NUM_CHANNELS);

    // Solve for each color channel
    for (int channel = 0; channel < NUM_CHANNELS; ++channel) {
        Eigen::VectorXd solution;
        switch (channel) {
            case 0: solution = linear_solver_.solve(boundary_red_);   break;
            case 1: solution = linear_solver_.solve(boundary_green_); break;
            case 2: solution = linear_solver_.solve(boundary_blue_);  break;
        }

        if (linear_solver_.info() != Eigen::Success) {
            throw std::runtime_error("Failed to solve linear system");
        }

        // Store results and clamp to valid range
        for (int i = 0; i < mask_width_ * mask_height_; ++i) {
            solvedValues[i * NUM_CHANNELS + channel] = 
                static_cast<unsigned char>(std::clamp(solution(i), 0.0, 255.0));
        }
    }

    // Copy results to output image
    const int sourceWidth = source_image()->width();
    const int sourceHeight = source_image()->height();
    const int targetWidth = target_image()->width();
    const int targetHeight = target_image()->height();

    for (int y = 0; y < mask_height_; ++y) {
        for (int x = 0; x < mask_width_; ++x) {
            int sourceX = x + source_offset_x();
            int sourceY = y + source_offset_y();
            
            if (sourceX >= 0 && sourceX < targetWidth && 
                sourceY >= 0 && sourceY < targetHeight) {
                int idx = (y * mask_width_ + x) * NUM_CHANNELS;
                result->set_pixel(
                    std::clamp(sourceX, 0, sourceWidth - 1),
                    std::clamp(sourceY, 0, sourceHeight - 1),
                    {solvedValues[idx], solvedValues[idx + 1], solvedValues[idx + 2]}
                );
            }
        }
    }
    
    return result;
}

} // namespace USTC_CG
