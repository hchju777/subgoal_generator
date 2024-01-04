cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MY_TARGET subgoal_generator)

project ( ${MY_TARGET} )

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

################################
# Normal Libraries & Executables
################################
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
find_package(CGAL REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories( ${YAML_CPP_INCUDE_DIRS})
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( ${Eigen3_INCLUDE_DIRS} )

include_directories("include")
file(GLOB SRC_FILES "src/*.cpp")
list(REMOVE_ITEM SRC_FILES ${PROJECT_SOURCE_DIR}/src/main.cpp)
add_library(src_lib ${SRC_FILES})

add_library(libraries INTERFACE)
target_link_libraries(libraries INTERFACE 
  src_lib
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CGAL_LIBRARIES}
  ${Eigen3_LIBS}
)

add_executable(${MY_TARGET} ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(${MY_TARGET} libraries)

################################
# Testing
################################
option(BUILD_TESTS "Build all tests." OFF)

if (BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip)
  FetchContent_MakeAvailable(googletest)

  enable_testing()

  include(GoogleTest)

  add_executable(bvc_gen_test test/bvc_generator_test.cpp)
  target_link_libraries(bvc_gen_test libraries)
  target_link_libraries(bvc_gen_test GTest::gtest_main)
  gtest_discover_tests(bvc_gen_test)

  add_executable(dynamic_graph_test test/dynamic_graph_test.cpp)
  target_link_libraries(dynamic_graph_test libraries)
  target_link_libraries(dynamic_graph_test GTest::gtest_main)
  gtest_discover_tests(dynamic_graph_test)

  add_executable(velocity_obstacle_test test/velocity_obstacle_test.cpp)
  target_link_libraries(velocity_obstacle_test libraries)
  target_link_libraries(velocity_obstacle_test GTest::gtest_main)
  gtest_discover_tests(velocity_obstacle_test)
endif()