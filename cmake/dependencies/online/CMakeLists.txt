cmake_minimum_required(VERSION 3.0.2)
project(online)

add_subdirectory(external_project)
if (${CMAKE_VERSION} VERSION_LESS 3.14)
  message(WARNING "Cannot use FetchContent with version ${CMAKE_VERSION}")
else()
  add_subdirectory(fetch_content)
endif()
