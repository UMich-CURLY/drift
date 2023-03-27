# Find google test
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
include(GoogleTest)
FetchContent_MakeAvailable(googletest)

set(test_prefix "TEST")
enable_testing()

add_subdirectory(test_example_1)
add_subdirectory(imumeasurement_test)
add_subdirectory(velocitymeasurement_test)
add_subdirectory(contactmeasurement_test)
add_subdirectory(jointstatemeasurement_test)
add_subdirectory(cheetahkinematics_test)
add_subdirectory(velocitycorrection_test)
