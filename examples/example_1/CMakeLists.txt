# Create the test binary
add_executable(example_1 example_1.cpp)
# Link tests
target_link_libraries(
  example_1
  math
)
