cmake_minimum_required(VERSION 3.8)
project(touch_haptics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(OPENHAPTICS_ROOT "" CACHE PATH "OpenHaptics install prefix (optional)")

find_path(OPENHAPTICS_INCLUDE_DIR HD/hd.h
  PATHS
    ${OPENHAPTICS_ROOT}/include
    /usr/include
    /usr/local/include
)

find_library(OPENHAPTICS_HD_LIB NAMES HD
  PATHS
    ${OPENHAPTICS_ROOT}/lib
    ${OPENHAPTICS_ROOT}/lib64
    /usr/lib
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
)

find_library(OPENHAPTICS_HDU_LIB NAMES HDU
  PATHS
    ${OPENHAPTICS_ROOT}/lib
    ${OPENHAPTICS_ROOT}/lib64
    /usr/lib
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
)

# ---------------------------------------------------------------------------
# Build options
# ---------------------------------------------------------------------------
# The simulator exists so the control path can be developed and validated with
# no hardware attached -- hardware time is the scarcest resource on this
# project. It replaces libHD only; the NODES it drives are byte-identical to
# the ones that run on the arms, which is the property that makes sim results
# worth anything.
option(TOUCH_HAPTICS_BUILD_SIM "Build the hardware-free simulator and *_sim nodes" ON)
option(TOUCH_HAPTICS_BUILD_HW  "Build hardware nodes (requires OpenHaptics)" ON)
# OFF by default on purpose: this is the gate that lets the real touch_slave
# binary actually write joint torque to the arms (C1's slave-force ->
# master-torque relay). Must be turned on explicitly with
# -DTOUCH_HAPTICS_ENABLE_REAL_TORQUE_WRITE=ON, a deliberate build-time
# decision, not something a routine `colcon build` should ever flip.
option(TOUCH_HAPTICS_ENABLE_REAL_TORQUE_WRITE "Compile the real touch_slave with its joint-torque write path armed (DANGEROUS: writes real torque to real motors)" OFF)

set(OPENHAPTICS_FOUND TRUE)
if(NOT OPENHAPTICS_INCLUDE_DIR OR NOT OPENHAPTICS_HD_LIB OR NOT OPENHAPTICS_HDU_LIB)
  set(OPENHAPTICS_FOUND FALSE)
endif()

# Previously an unconditional FATAL_ERROR, which meant the repo could not be
# built, tested or even syntax-checked on any machine without OpenHaptics
# installed. Now it is fatal only when hardware targets are actually wanted and
# the simulator is not available as a fallback.
if(NOT OPENHAPTICS_FOUND)
  if(TOUCH_HAPTICS_BUILD_HW AND NOT TOUCH_HAPTICS_BUILD_SIM)
    message(FATAL_ERROR
      "OpenHaptics not found (need hd.h, libHD, libHDU) and TOUCH_HAPTICS_BUILD_SIM is OFF. "
      "Set -DOPENHAPTICS_ROOT=/path, or enable the simulator with -DTOUCH_HAPTICS_BUILD_SIM=ON.")
  endif()
  message(WARNING
    "OpenHaptics not found -- skipping hardware nodes. Simulator targets will still build. "
    "Set -DOPENHAPTICS_ROOT=/path to build against real devices.")
  set(TOUCH_HAPTICS_BUILD_HW OFF)
endif()

# ---------------------------------------------------------------------------
# Hardware nodes
# ---------------------------------------------------------------------------
function(add_touch_node target_name source_file)
  add_executable(${target_name} src/${source_file})
  ament_target_dependencies(${target_name} rclcpp geometry_msgs std_msgs)
  target_include_directories(${target_name} PRIVATE
    ${OPENHAPTICS_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(${target_name} ${OPENHAPTICS_HD_LIB} ${OPENHAPTICS_HDU_LIB})
  install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
  # Real-hardware-only marker, distinct from touch_slave_sim (add_touch_sim_node,
  # below) even when both are built with TOUCH_DOB_ENABLE_TORQUE_WRITE=1 -- used
  # to gate the HD_CURRENT_JOINT_TORQUE unit conversion (real OpenHaptics wants
  # mN*m per the vendor reference guide's Table 6-2; the sim's own hdSetDoublev/
  # hdGetDoublev stand-in was built to N*m and must NOT get this conversion, or
  # it silently breaks self-consistency with its own physics). See the 2026-09-03
  # comment at touch_slave.cpp's hdSetDoublev(HD_CURRENT_JOINT_TORQUE, ...) calls.
  target_compile_definitions(${target_name} PRIVATE TOUCH_HAPTICS_REAL_HD_HARDWARE=1)
endfunction()

if(TOUCH_HAPTICS_BUILD_HW)
  add_touch_node(touch_publisher touch_publisher.cpp)
  add_touch_node(touch_slave touch_slave.cpp)
  if(TOUCH_HAPTICS_ENABLE_REAL_TORQUE_WRITE)
    message(WARNING
      "TOUCH_HAPTICS_ENABLE_REAL_TORQUE_WRITE=ON: the real touch_slave "
      "binary will be able to write joint torque to real motors when "
      "joint_space_control:=true. Confirm dob_torque_f_max_n is set low "
      "and an e-stop is ready before running it.")
    target_compile_definitions(touch_slave PRIVATE TOUCH_DOB_ENABLE_TORQUE_WRITE=1)
  endif()
endif()

# ---------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------
if(TOUCH_HAPTICS_BUILD_SIM)
  add_library(hd_sim SHARED sim/src/hd_sim_api.cpp)
  # Prefer the REAL OpenHaptics headers when present: enum values and struct
  # layouts are then guaranteed identical to hardware, and the simulator is
  # substituting only the library. The vendored fallback is for machines with
  # no OpenHaptics at all.
  if(OPENHAPTICS_FOUND)
    target_include_directories(hd_sim PUBLIC ${OPENHAPTICS_INCLUDE_DIR})
  else()
    target_include_directories(hd_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/sim/include)
  endif()
  target_include_directories(hd_sim PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/sim/src)
  target_link_libraries(hd_sim pthread)
  install(TARGETS hd_sim DESTINATION lib)

  # Same source, same node, simulated device beneath it.
  function(add_touch_sim_node target_name source_file)
    add_executable(${target_name} src/${source_file})
    ament_target_dependencies(${target_name} rclcpp geometry_msgs std_msgs)
    target_include_directories(${target_name} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${target_name} hd_sim)
    install(TARGETS ${target_name} DESTINATION lib/${PROJECT_NAME})
  endfunction()

  add_touch_sim_node(touch_slave_sim touch_slave.cpp)

  # Opens the joint-torque write path (0.5) ONLY on the sim executable. This is
  # the entire safety argument for sim-only bilateral testing: touch_slave
  # (hardware) never receives this define, so DobConfig::writeAuthorized()'s
  # #else branch -- the only branch that can ever return true -- does not
  # exist in that binary's machine code. No parameter, launch flag, or
  # environment variable can reach code that was never compiled in. See
  # touch_dob.hpp's writeAuthorized() and touch_slave.cpp's configureBilateral().
  target_compile_definitions(touch_slave_sim PRIVATE TOUCH_DOB_ENABLE_TORQUE_WRITE=1)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DobDiagnostics.msg"
  "msg/ForceEstimate.msg"
  "msg/BilateralStatus.msg"
  DEPENDENCIES std_msgs
)

# The nodes use the interfaces this same package generates, so they must link
# against the generated C++ typesupport explicitly.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
foreach(_t touch_slave touch_slave_sim)
  if(TARGET ${_t})
    target_link_libraries(${_t} "${cpp_typesupport_target}")
  endif()
endforeach()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
# tests/test_touch_dob.cpp already held 55 assertions but was built by a
# hand-written g++ line in its own header comment and was wired into nothing.
# That meant it could rot silently. It is framework-free by design (so it stays
# runnable anywhere with a bare compiler), so it needs no gtest -- only an
# add_test that runs it.
if(BUILD_TESTING)
  foreach(_test test_touch_dob)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/${_test}.cpp)
      add_executable(${_test} tests/${_test}.cpp)
      target_include_directories(${_test} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
      add_test(NAME ${_test} COMMAND ${_test})
    endif()
  endforeach()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
