cmake_minimum_required(VERSION 2.8.3)
project(classes_basic)

include_directories(include)

add_executable(rectangle_test
    src/rectangle_main.cpp
    src/rectangle.cpp
    include/rectangle.hpp
    )

add_executable(circle_test
    src/circle_main.cpp
    )
