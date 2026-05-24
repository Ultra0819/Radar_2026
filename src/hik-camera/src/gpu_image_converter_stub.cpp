#include "hik_camera/gpu_image_converter.hpp"

namespace hik_camera
{

struct GpuImageConverter::Impl
{
  std::string last_error = "CUDA support was not built into hik_camera";
};

GpuImageConverter::GpuImageConverter() : impl_(new Impl) {}

GpuImageConverter::~GpuImageConverter() = default;

bool GpuImageConverter::available() const { return false; }

const std::string & GpuImageConverter::lastError() const { return impl_->last_error; }

bool GpuImageConverter::convertBayer8ToRgb8(
  const uint8_t *, std::size_t, uint32_t, uint32_t, uint32_t, uint8_t *, std::size_t)
{
  return false;
}

bool GpuImageConverter::isSupportedBayer8(uint32_t) { return false; }

}  // namespace hik_camera
