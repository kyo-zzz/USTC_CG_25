#pragma once
#include "clone_method.h"

namespace USTC_CG
{
class Paste : public CloneMethod {
 public:
  using CloneMethod::CloneMethod;
  ~Paste() override = default;
  std::shared_ptr<Image> solve() override;
};
}  // namespace USTC_CG