# cmake/eigen.cmake
include(fetch_content)

FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_PROGRESS ON
)

FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
  message(STATUS "Fetching Eigen library...")
  FetchContent_Populate(eigen)
  include_directories(${eigen_SOURCE_DIR})
endif()

FetchContent_MakeAvailable(eigen)