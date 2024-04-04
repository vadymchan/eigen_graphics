cmake_minimum_required(VERSION 3.10.0)
project(eigen_graphics)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/third_party)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
  FetchContent_Populate(eigen)
  include_directories(${eigen_SOURCE_DIR})
endif()

FetchContent_MakeAvailable(eigen)

# Not included in eigen v3.4.0 );
# Copy .clang-format file from Eigen source code root directory
# configure_file(${eigen_SOURCE_DIR}/.clang-format ${CMAKE_SOURCE_DIR}/.clang-format COPYONLY)

file(GLOB_RECURSE EIGEN_GRAPHICS_HEADERS "include/eigen_graphics/*.h")
file(GLOB_RECURSE LIB_SOURCES "src/*.cpp")

if(LIB_SOURCES)
  # Static Library
  message(STATUS "Build ${PROJECT_NAME} as static library")
  add_library(${PROJECT_NAME} STATIC ${EIGEN_GRAPHICS_HEADERS} ${LIB_SOURCES})
  target_include_directories(${PROJECT_NAME} PUBLIC include)
  target_include_directories(${PROJECT_NAME} PRIVATE src)
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${eigen_SOURCE_DIR})
  else()
  # Header Only
  message(STATUS "Build ${PROJECT_NAME} as header only library")
  add_library(${PROJECT_NAME} INTERFACE ${EIGEN_GRAPHICS_HEADERS})
  target_include_directories(${PROJECT_NAME} INTERFACE include)
  target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE ${eigen_SOURCE_DIR})
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
  endforeach()
endif()