cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MY_TARGET subgoal_generator)

project ( ${MY_TARGET} )

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

################################
# Normal Libraries & Executables
################################
include_directories("include")
file(GLOB SOURCES "src/*.cpp")
add_executable( ${MY_TARGET} ${SOURCES})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCUDE_DIRS})
target_link_libraries( ${MY_TARGET} ${YAML_CPP_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(${MY_TARGET} ${Boost_LIBRARIES})

################################
# Testing
################################
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

enable_testing()

include(GoogleTest)

add_executable(gtest_test test/gtestTest.cpp)
target_link_libraries(gtest_test GTest::gtest_main)
gtest_discover_tests(gtest_test)