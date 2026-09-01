cmake_minimum_required(VERSION 3.0.2)
project(traj_opt)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  mapping
  roscpp
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS mapping roscpp
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cc
  src/${PROJECT_NAME}_fake.cc
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_visibility_ray
    test/test_visibility_ray.test
    test/test_visibility_ray.cpp
  )
  if(TARGET test_visibility_ray)
    target_link_libraries(test_visibility_ray
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
endif()
