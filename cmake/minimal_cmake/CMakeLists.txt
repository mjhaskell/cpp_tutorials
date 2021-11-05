cmake_minimum_required(VERSION 3.0.2)
project(minimal_cmake)

add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)

