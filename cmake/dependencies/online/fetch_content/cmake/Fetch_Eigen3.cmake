cmake_minimum_required(VERSION 3.14)

include(FetchContent)

FetchContent_Declare(eigen_ext
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.3.7
)
FetchContent_MakeAvailable(eigen_ext)
set(EIGEN3_INCLUDE_DIRS ${eigen_ext_SOURCE_DIR})
