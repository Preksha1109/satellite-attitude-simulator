cmake_minimum_required(VERSION 3.14)
project(satellite_attitude_simulator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# --- core library -----------------------------------------------------------
add_library(attitude_core
  src/torque_models.cpp
  src/integrator.cpp
  src/simulator.cpp)
target_include_directories(attitude_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(MSVC)
  target_compile_options(attitude_core PRIVATE /W4)
else()
  target_compile_options(attitude_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# --- command-line simulator ---------------------------------------------------
add_executable(attitude_sim src/main.cpp)
target_link_libraries(attitude_sim PRIVATE attitude_core)

# --- example: custom torque model ---------------------------------------------
add_executable(custom_torque examples/custom_torque.cpp)
target_link_libraries(custom_torque PRIVATE attitude_core)

# --- tests ---------------------------------------------------------------------
include(CTest)
if(BUILD_TESTING)
  add_executable(attitude_tests tests/test_attitude.cpp)
  target_link_libraries(attitude_tests PRIVATE attitude_core)
  add_test(NAME attitude_tests COMMAND attitude_tests) 
add_executable(test_magnetorquer_rw tests/test_magnetorquer_rw.cpp)
target_link_libraries(test_magnetorquer_rw PRIVATE attitude_core)
add_test(NAME magnetorquer_rw_tests COMMAND test_magnetorquer_rw)

endif()
