cmake_minimum_required(VERSION 3.0.2)
project(submodule_dep_test)

add_subdirectory(lib/eigen)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  lib/eigen
  )
