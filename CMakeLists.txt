cmake_minimum_required(VERSION 3.1)
project(polygon)


set(CMAKE_CXX_STANDARD 20)

# Define version
set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 1)
set(PROJECT_VERSION_PATCH 0)


include_directories(src)
add_subdirectory(third_party_libs/googletest)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(${gmock_SOURCE_DIR}/include ${gmock_SOURCE_DIR})


enable_testing()
add_executable(line_test test/line_test.cc )
add_test(line_test line_test)
target_link_libraries(line_test gtest gtest_main gmock_main)
set_property(TARGET line_test PROPERTY CXX_STANDARD 20)

add_executable(polygon_test test/polygon_test.cc )
add_test(polygon_test polygon_test)
target_link_libraries(polygon_test gtest gtest_main gmock_main)
set_property(TARGET polygon_test PROPERTY CXX_STANDARD 20)
