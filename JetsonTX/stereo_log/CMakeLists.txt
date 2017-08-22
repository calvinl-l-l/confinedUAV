cmake_minimum_required(VERSION 2.8)

PROJECT(stereoLOG)

file(
  GLOB cam_SRC
  "${PROJECT_SOURCE_DIR}/src/*.cpp"
  )

set(CMAKE_CXX_FLAGS "-std=c++11")

##For Jetson, OpenCV4Tegra is based on OpenCV2.4
exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" )
   SET(VERSION_REQ_OCV "2.4")
   SET(VERSION_REQ_CUDA "")
   link_directories("/usr/lib/aarch64-linux-gnu/tegra") ## on Jetson TX1 64bits, the correct libGL.so is located here (the default one will lead to linking error (undef ref))

   unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
   option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
   message ("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")
   ##to prevent from opencv_dep_cudart dependencies error...
   ## cmake with -DCUDA_USE_STATIC_CUDA_RUNTIME=false can also be called.
endif()

find_package(OpenCV ${VERSION_REQ_OCV} REQUIRED)
find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)
find_package(Threads REQUIRED)

add_executable(stereoLOG ${cam_SRC})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

include_directories("${PROJECT_SOURCE_DIR}/src/")
include_directories("${OpenCV_INCLUDE_DIRS}")

target_link_libraries(stereoLOG ${OpenCV_LIBS} -lpthread)
