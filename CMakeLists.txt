PROJECT(motors_weg_cvw300)
cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_STANDARD 11)
SET (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.orogen/config")
INCLUDE(motors_weg_cvw300Base)

if (ROCK_TEST_ENABLED)
    enable_testing()
    find_package(Syskit REQUIRED)
    syskit_orogen_tests(test)
endif()
