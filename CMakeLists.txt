cmake_minimum_required(VERSION 3.5)
#######################
project(UR)


include_directories(
  include/
  /usr/include/eigen3
)

find_package(ur_rtde REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  thread
  program_options
)
find_library(YAMLLib yaml-cpp HINTS /usr/local/lib)

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")
add_library(UR_RTDE SHARED ${SOURCE_FILES})
target_link_libraries(UR_RTDE yaml-cpp ur_rtde::rtde
)

# test
add_executable(test_ur_rtde test/test.cpp)
target_link_libraries(test_ur_rtde UR_RTDE  ur_rtde::rtde
)

