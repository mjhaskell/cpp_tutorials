cmake_minimum_required(VERSION 2.8.3)
project(templates)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

include_directories(include)

add_executable(template_class_testing
  src/template_classes/main.cpp
)

add_executable(template_function_testing
  src/template_functions/main.cpp
)

add_executable(template_variable_testing
  src/template_variables/main.cpp
)
set_property(TARGET template_variable_testing PROPERTY CXX_STANDARD 14)
