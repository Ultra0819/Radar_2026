#ifndef HIK_CAMERA_GPU_IMAGE_CONVERTER_HPP_
#define HIK_CAMERA_GPU_IMAGE_CONVERTER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace hik_camera
{

class GpuImageConverter
{
public:
  GpuImageConverter();
  ~GpuImageConverter();

  GpuImageConverter(const GpuImageConverter &) = delete;
  GpuImageConverter & operator=(const GpuImageConverter &) = delete;

  bool available() const;
  const std::string & lastError() const;

  bool convertBayer8ToRgb8(
    const uint8_t * src, std::size_t src_size, uint32_t width, uint32_t height, uint32_t pixel_type,
    uint8_t * dst, std::size_t dst_size);

  static bool isSupportedBayer8(uint32_t pixel_type);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hik_camera

#endif  // HIK_CAMERA_GPU_IMAGE_CONVERTER_HPP_
