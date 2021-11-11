cmake_minimum_required(VERSION 3.0.2)
project(libraries)

add_library(hello STATIC src/hello.cpp)
target_include_directories(hello PUBLIC include)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} hello)
