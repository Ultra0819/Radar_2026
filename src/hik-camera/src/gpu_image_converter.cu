#include <cuda_runtime.h>

#include <limits>
#include <sstream>

#include "PixelType.h"
#include "hik_camera/gpu_image_converter.hpp"

namespace
{

constexpr int kBayerGR = 0;
constexpr int kBayerRG = 1;
constexpr int kBayerGB = 2;
constexpr int kBayerBG = 3;

int patternFromPixelType(uint32_t pixel_type)
{
  switch (pixel_type) {
    case PixelType_Gvsp_BayerGR8:
    case PixelType_Gvsp_HB_BayerGR8:
      return kBayerGR;
    case PixelType_Gvsp_BayerRG8:
    case PixelType_Gvsp_HB_BayerRG8:
      return kBayerRG;
    case PixelType_Gvsp_BayerGB8:
    case PixelType_Gvsp_HB_BayerGB8:
      return kBayerGB;
    case PixelType_Gvsp_BayerBG8:
    case PixelType_Gvsp_HB_BayerBG8:
      return kBayerBG;
    default:
      return -1;
  }
}

std::string cudaErrorMessage(const char * operation, cudaError_t error)
{
  std::ostringstream oss;
  oss << operation << " failed: " << cudaGetErrorString(error) << " (" << static_cast<int>(error)
      << ")";
  return oss.str();
}

__device__ __forceinline__ int clampCoord(int value, int upper)
{
  if (value < 0) {
    return 0;
  }
  if (value >= upper) {
    return upper - 1;
  }
  return value;
}

__device__ __forceinline__ unsigned char loadBayer(
  const unsigned char * src, int width, int height, int x, int y)
{
  x = clampCoord(x, width);
  y = clampCoord(y, height);
  return src[y * width + x];
}

__device__ __forceinline__ bool isRedSite(int pattern, int x, int y)
{
  const bool even_x = (x & 1) == 0;
  const bool even_y = (y & 1) == 0;

  if (pattern == kBayerRG) {
    return even_x && even_y;
  }
  if (pattern == kBayerBG) {
    return !even_x && !even_y;
  }
  if (pattern == kBayerGR) {
    return !even_x && even_y;
  }
  return even_x && !even_y;
}

__device__ __forceinline__ bool isBlueSite(int pattern, int x, int y)
{
  const bool even_x = (x & 1) == 0;
  const bool even_y = (y & 1) == 0;

  if (pattern == kBayerRG) {
    return !even_x && !even_y;
  }
  if (pattern == kBayerBG) {
    return even_x && even_y;
  }
  if (pattern == kBayerGR) {
    return even_x && !even_y;
  }
  return !even_x && even_y;
}

__device__ __forceinline__ unsigned char avg2(unsigned char a, unsigned char b)
{
  return static_cast<unsigned char>((static_cast<int>(a) + static_cast<int>(b) + 1) / 2);
}

__device__ __forceinline__ unsigned char avg4(
  unsigned char a, unsigned char b, unsigned char c, unsigned char d)
{
  return static_cast<unsigned char>(
    (static_cast<int>(a) + static_cast<int>(b) + static_cast<int>(c) + static_cast<int>(d) + 2) /
    4);
}

__global__ void bayer8ToRgb8Kernel(
  const unsigned char * src, unsigned char * dst, int width, int height, int pattern)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);

  if (x >= width || y >= height) {
    return;
  }

  unsigned char r = 0;
  unsigned char g = 0;
  unsigned char b = 0;

  const unsigned char center = loadBayer(src, width, height, x, y);

  if (isRedSite(pattern, x, y)) {
    r = center;
    g = avg4(
      loadBayer(src, width, height, x - 1, y), loadBayer(src, width, height, x + 1, y),
      loadBayer(src, width, height, x, y - 1), loadBayer(src, width, height, x, y + 1));
    b = avg4(
      loadBayer(src, width, height, x - 1, y - 1), loadBayer(src, width, height, x + 1, y - 1),
      loadBayer(src, width, height, x - 1, y + 1), loadBayer(src, width, height, x + 1, y + 1));
  } else if (isBlueSite(pattern, x, y)) {
    r = avg4(
      loadBayer(src, width, height, x - 1, y - 1), loadBayer(src, width, height, x + 1, y - 1),
      loadBayer(src, width, height, x - 1, y + 1), loadBayer(src, width, height, x + 1, y + 1));
    g = avg4(
      loadBayer(src, width, height, x - 1, y), loadBayer(src, width, height, x + 1, y),
      loadBayer(src, width, height, x, y - 1), loadBayer(src, width, height, x, y + 1));
    b = center;
  } else {
    const bool horizontal_red = ((pattern == kBayerRG || pattern == kBayerGR) && ((y & 1) == 0)) ||
                                ((pattern == kBayerBG || pattern == kBayerGB) && ((y & 1) != 0));

    g = center;
    if (horizontal_red) {
      r = avg2(loadBayer(src, width, height, x - 1, y), loadBayer(src, width, height, x + 1, y));
      b = avg2(loadBayer(src, width, height, x, y - 1), loadBayer(src, width, height, x, y + 1));
    } else {
      r = avg2(loadBayer(src, width, height, x, y - 1), loadBayer(src, width, height, x, y + 1));
      b = avg2(loadBayer(src, width, height, x - 1, y), loadBayer(src, width, height, x + 1, y));
    }
  }

  const int dst_index = (y * width + x) * 3;
  dst[dst_index] = r;
  dst[dst_index + 1] = g;
  dst[dst_index + 2] = b;
}

}  // namespace

namespace hik_camera
{

struct GpuImageConverter::Impl
{
  bool available = false;
  std::string last_error;
  unsigned char * device_src = nullptr;
  unsigned char * device_dst = nullptr;
  std::size_t src_capacity = 0;
  std::size_t dst_capacity = 0;
  cudaStream_t stream = nullptr;

  Impl()
  {
    cudaError_t flag_error = cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
    if (flag_error == cudaErrorSetOnActiveProcess) {
      cudaGetLastError();
    }

    int device_count = 0;
    cudaError_t error = cudaGetDeviceCount(&device_count);
    if (error != cudaSuccess) {
      last_error = cudaErrorMessage("cudaGetDeviceCount", error);
      return;
    }
    if (device_count <= 0) {
      last_error = "no CUDA-capable device found";
      return;
    }

    error = cudaSetDevice(0);
    if (error != cudaSuccess) {
      last_error = cudaErrorMessage("cudaSetDevice", error);
      return;
    }

    error = cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking);
    if (error != cudaSuccess) {
      last_error = cudaErrorMessage("cudaStreamCreateWithFlags", error);
      return;
    }

    available = true;
  }

  ~Impl()
  {
    if (device_src != nullptr) {
      cudaFree(device_src);
    }
    if (device_dst != nullptr) {
      cudaFree(device_dst);
    }
    if (stream != nullptr) {
      cudaStreamDestroy(stream);
    }
  }

  bool reserve(unsigned char ** buffer, std::size_t * capacity, std::size_t required)
  {
    if (*capacity >= required) {
      return true;
    }

    if (*buffer != nullptr) {
      cudaFree(*buffer);
      *buffer = nullptr;
      *capacity = 0;
    }

    cudaError_t error = cudaMalloc(reinterpret_cast<void **>(buffer), required);
    if (error != cudaSuccess) {
      last_error = cudaErrorMessage("cudaMalloc", error);
      return false;
    }

    *capacity = required;
    return true;
  }
};

GpuImageConverter::GpuImageConverter() : impl_(new Impl) {}

GpuImageConverter::~GpuImageConverter() = default;

bool GpuImageConverter::available() const { return impl_->available; }

const std::string & GpuImageConverter::lastError() const { return impl_->last_error; }

bool GpuImageConverter::convertBayer8ToRgb8(
  const uint8_t * src, std::size_t src_size, uint32_t width, uint32_t height, uint32_t pixel_type,
  uint8_t * dst, std::size_t dst_size)
{
  if (!impl_->available) {
    return false;
  }

  if (src == nullptr || dst == nullptr) {
    impl_->last_error = "source or destination buffer is null";
    return false;
  }

  if (width == 0 || height == 0) {
    impl_->last_error = "image width or height is zero";
    return false;
  }

  if (
    width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
    height > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
    impl_->last_error = "image dimensions exceed CUDA kernel limits";
    return false;
  }

  const int pattern = patternFromPixelType(pixel_type);
  if (pattern < 0) {
    impl_->last_error = "unsupported Bayer8 pixel type";
    return false;
  }

  const std::size_t src_required = static_cast<std::size_t>(width) * height;
  const std::size_t dst_required = src_required * 3;
  if (src_size < src_required || dst_size < dst_required) {
    impl_->last_error = "source or destination buffer is too small";
    return false;
  }

  if (
    !impl_->reserve(&impl_->device_src, &impl_->src_capacity, src_required) ||
    !impl_->reserve(&impl_->device_dst, &impl_->dst_capacity, dst_required)) {
    return false;
  }

  cudaError_t error =
    cudaMemcpyAsync(impl_->device_src, src, src_required, cudaMemcpyHostToDevice, impl_->stream);
  if (error != cudaSuccess) {
    impl_->last_error = cudaErrorMessage("cudaMemcpyAsync H2D", error);
    return false;
  }

  const dim3 block(16, 16);
  const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
  bayer8ToRgb8Kernel<<<grid, block, 0, impl_->stream>>>(
    impl_->device_src, impl_->device_dst, static_cast<int>(width), static_cast<int>(height),
    pattern);

  error = cudaGetLastError();
  if (error != cudaSuccess) {
    impl_->last_error = cudaErrorMessage("bayer8ToRgb8Kernel", error);
    return false;
  }

  error =
    cudaMemcpyAsync(dst, impl_->device_dst, dst_required, cudaMemcpyDeviceToHost, impl_->stream);
  if (error != cudaSuccess) {
    impl_->last_error = cudaErrorMessage("cudaMemcpyAsync D2H", error);
    return false;
  }

  error = cudaStreamSynchronize(impl_->stream);
  if (error != cudaSuccess) {
    impl_->last_error = cudaErrorMessage("cudaStreamSynchronize", error);
    return false;
  }

  return true;
}

bool GpuImageConverter::isSupportedBayer8(uint32_t pixel_type)
{
  return patternFromPixelType(pixel_type) >= 0;
}

}  // namespace hik_camera
