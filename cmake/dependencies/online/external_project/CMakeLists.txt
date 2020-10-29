cmake_minimum_required(VERSION 3.0.2)
project(external_dep_test)

include(cmake/External_Eigen3.cmake)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  )
add_dependencies(${PROJECT_NAME} eigen_ext)
