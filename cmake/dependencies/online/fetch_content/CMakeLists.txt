cmake_minimum_required(VERSION 3.14)
project(fetch_dep_test)

include(cmake/Fetch_Eigen3.cmake)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  )
