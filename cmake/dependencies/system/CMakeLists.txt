cmake_minimum_required(VERSION 3.0.2)
project(system_dep_test)

find_package(Eigen3 REQUIRED 3.3.7)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  )
