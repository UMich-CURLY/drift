file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
# Create the test binary
add_executable(${test_prefix}_contactmeasurement_test contactmeasurement_test.cpp ${SRC_FILES})
# Link tests

target_link_libraries(
  ${test_prefix}_contactmeasurement_test
  GTest::gtest_main
)
gtest_discover_tests(${test_prefix}_contactmeasurement_test)