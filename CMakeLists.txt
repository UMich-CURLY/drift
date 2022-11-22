cmake_minimum_required(VERSION 3.14)

project(curly_state_estimator VERSION 0.1 DESCRIPTION "An InEKF based state estimator")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

option(BUILD_EXAMPLES "Build the examples" ON)
option(BUILD_TESTS "Build the test files" ON)
option(BUILD_DOC "Build Doxygen documents" ON)

# #################################################
# Find External Packages             #
# #################################################
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

# #################################################
# Include Packages                #
# #################################################
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include(FetchContent)

# #################################################
# Project Include Dir              #
# #################################################
include_directories(${PROJECT_SOURCE_DIR}/include)

# #################################################
# Create Library                 #
# #################################################
file(GLOB inekf
  "${PROJECT_SOURCE_DIR}/src/inekf/*.cpp"
)
add_library(inekf ${inekf})
add_library(robot_state ${PROJECT_SOURCE_DIR}/src/state/robot_state.cpp)
add_library(utils ${PROJECT_SOURCE_DIR}/src/utils/utils.cpp)
add_library(state_estimator ${PROJECT_SOURCE_DIR}/src/state_estimator.cpp)
add_library(lie_group ${PROJECT_SOURCE_DIR}/src/math/lie_group.cpp)

# #################################################
# Build Examples                 #
# #################################################
if(BUILD_EXAMPLES)
  message("Building examples...")
  add_subdirectory(examples)
endif(BUILD_EXAMPLES)

# #################################################
# Build Tests                  #
# #################################################
if(BUILD_TESTS)
  message("Building tests...")
  add_subdirectory(tests)
endif(BUILD_TESTS)

# #################################################
# Build Documents                #
# #################################################
if(BUILD_DOC)
  # find doxygen
  find_package(Doxygen QUIET)

  if(DOXYGEN_FOUND)
    message("Building documents...")

    # set input and output files
    set(DOXYGEN_IN ${PROJECT_SOURCE_DIR}/doc/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configu re the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    # note the option ALL which allows to build the docs together with the application
    add_custom_target(doc_doxygen ALL
      COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating API documentation with Doxygen"
      VERBATIM)
  else(DOXYGEN_FOUND)
    message(WARNING "Doxygen need to be installed to generate the doxygen documentation")
  endif(DOXYGEN_FOUND)
endif(BUILD_DOC)

# #################################################
# Add Executables                #
# #################################################
add_executable(curly_state_estimator src/main.cpp)
target_link_libraries(curly_state_estimator inekf lie_group robot_state utils)
