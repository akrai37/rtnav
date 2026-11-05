cmake_minimum_required(VERSION 3.20)
project(rtnav VERSION 0.1.0 LANGUAGES C)

# Host build: the portable navigation logic, the robot simulation and the
# tests. The AVR firmware itself is built by firmware/Makefile with avr-gcc.

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(RTNAV_WERROR "Treat compiler warnings as errors" OFF)
option(RTNAV_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(RTNAV_BUILD_TESTS "Build the tests" ON)

if(RTNAV_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=address,undefined)
endif()

add_compile_definitions(_POSIX_C_SOURCE=200809L)

add_library(rtnav_warnings INTERFACE)
target_compile_options(rtnav_warnings INTERFACE
  -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion
  -Wstrict-prototypes -Wmissing-prototypes -Wimplicit-fallthrough -Wformat=2 -Wcast-qual -Wvla)
if(RTNAV_WERROR)
  target_compile_options(rtnav_warnings INTERFACE -Werror)
endif()

# The navigation logic, compiled unchanged for the host here and for the
# AVR by firmware/Makefile.
add_library(rtnav_nav src/nav.c)
target_include_directories(rtnav_nav PUBLIC include)
target_link_libraries(rtnav_nav PRIVATE rtnav_warnings)

# Closed-loop simulation of the robot, host only.
add_library(rtnav_sim sim/sim.c sim/track.c)
target_include_directories(rtnav_sim PUBLIC sim)
target_link_libraries(rtnav_sim PUBLIC rtnav_nav PRIVATE rtnav_warnings)
find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
  target_link_libraries(rtnav_sim PUBLIC ${MATH_LIBRARY})
endif()

add_executable(rtnav-sim tools/rtnav_sim.c)
target_link_libraries(rtnav-sim PRIVATE rtnav_sim rtnav_warnings)

if(RTNAV_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
