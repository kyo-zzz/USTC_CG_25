#include "Paste.h"

namespace USTC_CG
{
std::shared_ptr<Image> Paste::solve() {
  // 直接返回源图像作为默认实现
  return source_image();
}
}  // namespace USTC_CG