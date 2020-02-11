cmake_minimum_required(VERSION 2.8.3)
project(learn_cpp)

find_package(Eigen3 REQUIRED)
#add_subdirectory(lib/<lib_dir_name>)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    )

add_executable(${PROJECT_NAME}
    main.cpp
    )
#target_link_libraries(${PROJECT_NAME}
#    ${EIGEN3_LIBRARIES}
#    )
