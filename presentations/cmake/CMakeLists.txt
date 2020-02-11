cmake_minimum_required(VERSION 2.8.3)
project(learn_cpp)

# specify libraries you want to use from system-wide installations
find_package(Eigen3 REQUIRED)

# specify libraries you want to use located in your lib folder
#add_subdirectory(lib/osqp-eigen)

# specify where you want to be able to look to #include header files
# put your own include directory as well as those from used libraries
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    )

file(GLOB_RECURSE PROJ_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    FOLLOW_SYMLINKS src/*.cpp)
file(GLOB_RECURSE PROJ_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    FOLLOW_SYMLINKS include/*.hpp)

add_executable(${PROJECT_NAME}
    ${PROJ_SRC}
    ${PROJ_INC}
    )
#target_link_libraries(${PROJECT_NAME}
#    )
