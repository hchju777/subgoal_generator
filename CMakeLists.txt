cmake_minimum_required(VERSION 2.8)

set(MY_TARGET subgoal_generator)

project ( ${MY_TARGET} )

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include_directories("include")
file(GLOB SOURCES "src/*.cpp")
add_executable( ${MY_TARGET} ${SOURCES})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCUDE_DIRS})
target_link_libraries( ${MY_TARGET} ${YAML_CPP_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(${MY_TARGET} ${Boost_LIBRARIES})