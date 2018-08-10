cmake_minimum_required(VERSION 3.0)

project (swiftlet)

#Project version number
set (swiftlet_VERSION_MAJOR 1)
set (swiftlet_VERSION_MINOR 0)

set(CMAKE_CXX_FLAGS "-std=c++11")


file(
  GLOB swiftlet_SRC
  "${PROJECT_SOURCE_DIR}/src/*.cpp"
  "${PROJECT_SOURCE_DIR}/lib/cserial/*.cpp"
  "${PROJECT_SOURCE_DIR}/lib/Scheduler/*.cpp"
  )

file(
  GLOB swiftlet_SLIB
  "${PROJECT_SOURCE_DIR}/lib/urg_library-1.2.0/src/*.so"
  "${PROJECT_SOURCE_DIR}/lib/urg_library-1.2.0/src/*.a"

  )

add_executable(swiftlet ${swiftlet_SRC})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(Threads REQUIRED)
find_package(Boost 1.6 COMPONENTS program_options thread system REQUIRED)

# include
include_directories("${PROJECT_SOURCE_DIR}/src/")
include_directories("${PROJECT_SOURCE_DIR}/lib/cserial")
include_directories("${PROJECT_SOURCE_DIR}/lib/Scheduler")
include_directories(${Boost_INCLUDE_DIR})


target_link_libraries(swiftlet ${Boost_LIBRARIES} ${swiftlet_SLIB} -lpthread -lwiringPi -lrt -lcrypt)

#target_compile_options(swiftlet -std=c++11)
