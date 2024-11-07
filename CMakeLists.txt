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
include(FetchContent) # Introduced since CMake 3.11

# #################################################
# Project Include Dir              #
# #################################################
include_directories(${PROJECT_SOURCE_DIR}/include)

# #################################################
# Create Library                 #
# #################################################
# Source file for mini cheetah robot:
file(GLOB MINI_CHEETAH_KIN_SRC_FILES ${PROJECT_SOURCE_DIR}/src/kinematics/robots/mini_cheetah/*.cpp)

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})
add_library(${LIBRARY_TARGET_NAME}
  ${PROJECT_SOURCE_DIR}/src/state/robot_state.cpp
  ${PROJECT_SOURCE_DIR}/src/estimator/inekf_estimator.cpp
  ${PROJECT_SOURCE_DIR}/src/math/lie_group.cpp
  ${PROJECT_SOURCE_DIR}/src/math/se_k_3.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/base_propagation.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/base_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/legged_kinematics_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/velocity_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/correction/position_correction.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/propagation/imu_propagation.cpp
  ${PROJECT_SOURCE_DIR}/src/filter/inekf/inekf.cpp
  ${PROJECT_SOURCE_DIR}/src/imu_filter/imu_ang_vel_ekf.cpp
  ${PROJECT_SOURCE_DIR}/src/measurement/measurement.cpp
  ${PROJECT_SOURCE_DIR}/src/measurement/legged_kinematics.cpp
  ${PROJECT_SOURCE_DIR}/src/kinematics/mini_cheetah_kinematics.cpp
  ${MINI_CHEETAH_KIN_SRC_FILES}
)
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
)

# #################################################
# Build Examples                 #
# #################################################
# if(BUILD_EXAMPLES)
# message("Building examples...")
# add_subdirectory(examples)
# endif(BUILD_EXAMPLES)

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
# INSTALL LIBRARY               #
# #################################################
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_INSTALL_PREFIX "/usr/local")

# List of H (header) library files.
file(GLOB ESTIMATOR_HEADER_FILES ${PROJECT_SOURCE_DIR}/include/drift/estimator/*.h)

# Filter related header files
file(GLOB FILTER_HEADER_FILES ${PROJECT_SOURCE_DIR}/include/drift/filter/*.h)
file(GLOB INEKF_FILTER_HEADER_FILES ${PROJECT_SOURCE_DIR}/include/drift/filter/inekf/*.h)
file(GLOB INEKF_CORRECTION_HEADER_FILES ${PROJECT_SOURCE_DIR}/include/drift/filter/inekf/correction/*.h)
file(GLOB INEKF_PROPAGATION_HEADER_FILES ${PROJECT_SOURCE_DIR}/include/drift/filter/inekf/propagation/*.h)

# Math header
file(GLOB MATH_HEADER_FILES ${PROJECT_SOURCE_DIR}/include/drift/math/*.h)

# State header
file(GLOB STATE_HEADER_FILES ${PROJECT_SOURCE_DIR}/include/drift/state/*.h)

# Measurements header
file(GLOB IMU_FILTER_HEADER_FILES ${PROJECT_SOURCE_DIR}/include/drift/imu_filter/*.h)
file(GLOB KINEMATICS_HEADER_FILES ${PROJECT_SOURCE_DIR}/include/drift/kinematics/*.h)
file(GLOB MEASUREMENT_HEADER_FILES ${PROJECT_SOURCE_DIR}/include/drift/measurement/*.h)
file(GLOB MEASUREMENT_IMPL_FILES ${PROJECT_SOURCE_DIR}/include/drift/measurement/impl/*.cpp)
file(GLOB MINI_CHEETAH_KIN_HEADER_FILES ${PROJECT_SOURCE_DIR}/include/drift/kinematics/robots/mini_cheetah/*.h)

# Utils header
file(GLOB UTILS_HEADER_FILES ${PROJECT_SOURCE_DIR}/include/drift/utils/*.h)

# Install the library
install(TARGETS ${LIBRARY_TARGET_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}"
)

# Install header files: Copying the header files to desired place
install(FILES ${ESTIMATOR_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/estimator)
install(FILES ${FILTER_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/filter)
install(FILES ${INEKF_FILTER_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/filter/inekf)
install(FILES ${INEKF_CORRECTION_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/filter/inekf/correction)
install(FILES ${INEKF_PROPAGATION_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/filter/inekf/propagation)

install(FILES ${MATH_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/math)
install(FILES ${STATE_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/state)

install(FILES ${IMU_FILTER_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/imu_filter)
install(FILES ${KINEMATICS_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/kinematics)
install(FILES ${MEASUREMENT_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/measurement)
install(FILES ${MEASUREMENT_IMPL_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/measurement/impl)
install(FILES ${MINI_CHEETAH_KIN_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/kinematics/robots/mini_cheetah)

install(FILES ${UTILS_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/utils)

install(

  # The export we want to save (matches name defined above containing the
  # install targets)
  EXPORT ${PROJECT_NAME}Targets

  # CMake file in which to store the export's information
  FILE ${PROJECT_NAME}Targets.cmake

  NAMESPACE drift::

  # where to place the resulting file (here, we're putting it with the library)
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

# generate the config file that includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/driftConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/driftConfig.cmake"
  INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/driftConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/driftConfigVersion.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# export(PACKAGE ${PROJECT_NAME})
export(
  TARGETS ${INSTALLABLE_LIBS}
  NAMESPACE drift::

  # EXPORT ${PROJECT_NAME}Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
)