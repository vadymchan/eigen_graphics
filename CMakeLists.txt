cmake_minimum_required(VERSION 3.10.0)

project(eigen_graphics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CONFIGURE_DEPENDS ON)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(fetch_content)
include(eigen)

file(GLOB_RECURSE EIGEN_GRAPHICS_HEADERS "include/eigen_graphics/*.h")
file(GLOB_RECURSE LIB_SOURCES "src/*.cpp")

if(LIB_SOURCES)
  # Static Library
  message(STATUS "Build ${PROJECT_NAME} as static library")

  add_library(${PROJECT_NAME} STATIC ${EIGEN_GRAPHICS_HEADERS} ${LIB_SOURCES})

  target_include_directories(${PROJECT_NAME} PUBLIC include)
  target_include_directories(${PROJECT_NAME} PRIVATE src)
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${eigen_SOURCE_DIR})

  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include/eigen_graphics" PREFIX "include" FILES ${EIGEN_GRAPHICS_HEADERS})
  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "src" FILES ${LIB_SOURCES})
else()
  # Header Only
  message(STATUS "Build ${PROJECT_NAME} as header only library")

  add_library(${PROJECT_NAME} INTERFACE ${EIGEN_GRAPHICS_HEADERS})

  target_include_directories(${PROJECT_NAME} INTERFACE include)
  target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE ${eigen_SOURCE_DIR})

  source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include/eigen_graphics" PREFIX "include" FILES ${EIGEN_GRAPHICS_HEADERS})
endif()

option(BUILD_EXAMPLES "Build examples" ON)

if(BUILD_EXAMPLES)
  file(GLOB_RECURSE EXAMPLE_SOURCES "examples/*.cpp")

  # Create an executable for each example source file
  foreach(EXAMPLE_SOURCE ${EXAMPLE_SOURCES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_SOURCE} NAME_WE)

    add_executable(${EXAMPLE_NAME} ${EXAMPLE_SOURCE})

    target_link_libraries(${EXAMPLE_NAME} PRIVATE ${PROJECT_NAME})
    target_include_directories(${EXAMPLE_NAME} PRIVATE ${eigen_SOURCE_DIR})
    target_include_directories(${EXAMPLE_NAME} PRIVATE include)

    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/examples" PREFIX "examples" FILES ${EXAMPLE_SOURCE})
  endforeach()
endif()