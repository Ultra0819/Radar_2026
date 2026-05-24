cmake_minimum_required(VERSION 3.8)
project(hik_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(HIK_CAMERA_SOURCES
  src/hik_camera_node.cpp
  src/video_player.cpp
)

if(NOT DEFINED CMAKE_CUDA_COMPILER)
  find_program(HIK_CAMERA_NVCC nvcc HINTS /usr/local/cuda/bin)
  if(HIK_CAMERA_NVCC)
    set(CMAKE_CUDA_COMPILER ${HIK_CAMERA_NVCC})
  endif()
endif()

find_package(CUDAToolkit QUIET)
set(HIK_CAMERA_USE_CUDA OFF)
if(CUDAToolkit_FOUND AND CMAKE_CUDA_COMPILER)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 75)
  endif()
  enable_language(CUDA)
  list(APPEND HIK_CAMERA_SOURCES src/gpu_image_converter.cu)
  set(HIK_CAMERA_USE_CUDA ON)
  message(STATUS "hik_camera: CUDA Toolkit found, enabling GPU Bayer converter")
else()
  list(APPEND HIK_CAMERA_SOURCES src/gpu_image_converter_stub.cpp)
  message(STATUS "hik_camera: CUDA compiler/toolkit not found, using CPU conversion fallback only")
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${HIK_CAMERA_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  hikSDK/include
)

target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-Wall -Werror>
)

if(HIK_CAMERA_USE_CUDA)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    CUDA_STANDARD 14
    CUDA_STANDARD_REQUIRED ON
    CUDA_SEPARABLE_COMPILATION OFF
  )
  target_compile_definitions(${PROJECT_NAME} PRIVATE HIK_CAMERA_USE_CUDA=1)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/amd64)
  install(
    DIRECTORY hikSDK/lib/amd64/
    DESTINATION lib
    PATTERN "libusb*" EXCLUDE  # 排除所有的 libusb 库文件
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/arm64)
  install(
    DIRECTORY hikSDK/lib/arm64/
    DESTINATION lib
    PATTERN "libusb*" EXCLUDE  # 排除所有的 libusb 库文件
  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME}
  MvCameraControl
  ${OpenCV_LIBS}
)

if(HIK_CAMERA_USE_CUDA)
  target_link_libraries(${PROJECT_NAME} CUDA::cudart)
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN hik_camera::HikCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(${PROJECT_NAME}
 PLUGIN fyt::camera_driver::VideoPlayerNode
 EXECUTABLE video_player_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)
