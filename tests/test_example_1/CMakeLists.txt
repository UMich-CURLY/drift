# Create the test binary
add_executable(${test_prefix}_test_example_1 test_example_1.cpp)

# Link tests
target_link_libraries(
  ${test_prefix}_test_example_1
  GTest::gtest_main
)

gtest_discover_tests(${test_prefix}_test_example_1)