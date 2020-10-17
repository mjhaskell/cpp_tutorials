cmake_minimum_required(VERSION 3.0.2)
project(minimum_cmake)

include_directories(include)

add_executable(${PROJECT_NAME}
    src/main.cpp
    include/hello.hpp
    )
