cmake_minimum_required(VERSION 3.14)

project(drift VERSION 0.1 DESCRIPTION "An InEKF based state estimator")

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
find_package(yaml-cpp REQUIRED)

message("Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message("Boost include dir: ${Boost_INCLUDE_DIRS}")
message("Yaml-cpp include dir: ${YAML_CPP_INCLUDE_DIR}")

# #################################################
# Include Packages                #
# #################################################
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})
include(FetchContent)

# #################################################
# Project Include Dir              #
# #################################################
include_directories(${PROJECT_SOURCE_DIR}/include)

# #################################################
# Create Library                 #
# #################################################
add_library(robot_state ${PROJECT_SOURCE_DIR}/src/state/robot_state.cpp)
add_library(inekf_estimator ${PROJECT_SOURCE_DIR}/src/estimator/inekf_estimator.cpp)
add_library(math ${PROJECT_SOURCE_DIR}/src/math/lie_group.cpp
  ${PROJECT_SOURCE_DIR}/src/math/se_k_3.cpp)
add_library(base_propagation ${PROJECT_SOURCE_DIR}/src/filter/base_propagation.cpp)
add_library(base_correction ${PROJECT_SOURCE_DIR}/src/filter/base_correction.cpp)
add_library(inekf ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/legged_kinematics_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/velocity_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/propagation/imu_propagation.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/inekf.cpp)
add_library(measurement ${PROJECT_SOURCE_DIR}/src/measurement/measurement.cpp
  ${PROJECT_SOURCE_DIR}/src/measurement/legged_kinematics.cpp)
file(GLOB MINI_CHEETAH_KIN_SRC_FILES ${PROJECT_SOURCE_DIR}/src/kinematics/robots/mini_cheetah/*.cpp)
add_library(robot_kinematics ${PROJECT_SOURCE_DIR}/src/kinematics/mini_cheetah_kinematics.cpp
  ${MINI_CHEETAH_KIN_SRC_FILES})

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
# Add Library                #
# #################################################
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# # declaire a c++ library
# add_library(drift SHARED
# ${PROJECT_SOURCE_DIR}/src/state/robot_state.cpp
# ${PROJECT_SOURCE_DIR}/src/math/se_k_3.cpp
# ${PROJECT_SOURCE_DIR}/src/estimator/inekf_estimator.cpp
# ${PROJECT_SOURCE_DIR}/src/math/lie_group.cpp
# ${PROJECT_SOURCE_DIR}/src/filter/base_propagation.cpp
# ${PROJECT_SOURCE_DIR}/src/filter/base_correction.cpp
# ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/legged_kinematics_correction.cpp
# ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/velocity_correction.cpp
# ${PROJECT_SOURCE_DIR}/src/filter/inekf/propagation/imu_propagation.cpp
# ${PROJECT_SOURCE_DIR}/src/filter/inekf/inekf.cpp
# ${PROJECT_SOURCE_DIR}/src/measurement/measurement.cpp
# ${PROJECT_SOURCE_DIR}/src/measurement/legged_kinematics.cpp
# ${PROJECT_SOURCE_DIR}/src/kinematics/mini_cheetah_kinematics.cpp
# ${MINI_CHEETAH_KIN_SRC_FILES}
# )
# target_include_directories(drift PRIVATE ${PROJECT_SOURCE_DIR}/include)
# set_target_properties(drift PROPERTIES LIBRARY_OUTPUT_NAME drift VERSION ${PACKAGE_VERSION} SOVERSION ${PACKAGE_VERSION_MAJOR})

# # Add version info to the library
# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(
# driftConfigVersion.cmake
# VERSION ${PACKAGE_VERSION}
# COMPATIBILITY AnyNewerVersion
# )

# # Install the library
# install(TARGETS drift
# EXPORT driftTargets
# LIBRARY DESTINATION lib
# ARCHIVE DESTINATION lib
# RUNTIME DESTINATION bin
# INCLUDES DESTINATION include
# PUBLIC_HEADER DESTINATION include
# )

# install(EXPORT driftTargets
# FILE driftTargets.cmake
# NAMESPACE drift::
# DESTINATION lib/cmake/drift
# )

# configure_file(driftConfig.cmake.in driftConfig.cmake @ONLY)
# install(FILES "${CMAKE_CURRENT_BINARY_DIR}/driftConfig.cmake"
# "${CMAKE_CURRENT_BINARY_DIR}/driftConfigVersion.cmake"
# DESTINATION lib/cmake/drift
# )