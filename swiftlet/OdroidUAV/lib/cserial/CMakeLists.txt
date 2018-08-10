cmake_minimum_required(VERSION 3.0)

project (tuna)

#Project version number
set (tuna_VERSION_MAJOR 1)
set (tuna_VERSION_MINOR 0)

set(CMAKE_CXX_FLAGS "-std=c++11")

INCLUDE(CMakeForceCompiler) 
CMAKE_FORCE_C_COMPILER(gcc GNU)
CMAKE_FORCE_CXX_COMPILER(g++ GNU)


file(
  GLOB tuna_SRC
  "${PROJECT_SOURCE_DIR}/*.cpp"
  )



add_executable(tuna ${tuna_SRC})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

find_package(Threads REQUIRED)


# include
include_directories("${PROJECT_SOURCE_DIR}/src/")
include_directories("${PROJECT_SOURCE_DIR}/include")


target_link_libraries(tuna -lpthread -lrt -lcrypt)

#target_compile_options(tuna -std=c++11)
