cmake_minimum_required(VERSION 3.0)

project (tuna)

#Project version number
set (tuna_VERSION_MAJOR 1)
set (tuna_VERSION_MINOR 0)

set(CMAKE_CXX_FLAGS "-std=c++11")


file(
  GLOB tuna_SRC
  "${PROJECT_SOURCE_DIR}/src/*.cpp"
  )

file(
  GLOB tuna_SLIB
  "${PROJECT_SOURCE_DIR}/lib/urg_library-1.2.0/src/*.so"
  "${PROJECT_SOURCE_DIR}/lib/urg_library-1.2.0/src/*.a"

  )

add_executable(tuna ${tuna_SRC})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

find_package(Threads REQUIRED)
find_package(Boost 1.6 COMPONENTS program_options thread system REQUIRED)

# include
include_directories("${PROJECT_SOURCE_DIR}/src/")
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories(${Boost_INCLUDE_DIR}) 


target_link_libraries(tuna ${Boost_LIBRARIES} ${tuna_SLIB} -lpthread -lwiringPi -lrt -lcrypt)

#target_compile_options(tuna -std=c++11)
