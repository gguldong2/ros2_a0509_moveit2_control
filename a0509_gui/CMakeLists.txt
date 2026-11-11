cmake_minimum_required(VERSION 3.16)
project(a0509_gui CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Controller GF02.x -> 2, GF03.x -> 3
set(DRCF_VERSION 2 CACHE STRING "Doosan controller framework version")
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/API-DRFL/include/DRFLEx.h")
  set(_drfl_default "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/API-DRFL")
else()
  set(_drfl_default "$ENV{HOME}/doosan_ws/API-DRFL")
endif()
set(DRFL_ROOT "${_drfl_default}" CACHE PATH "API-DRFL clone")
set(DRFL_LIB_DIR "${DRFL_ROOT}/library/Linux/64bits/amd64/22.04")

find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Offline unit test for the MoveL pose math (RPY <-> ZYZ, relative targets). No robot needed.
add_executable(pose_math_test pose_math_test.cpp)
enable_testing()
add_test(NAME pose_math_test COMMAND pose_math_test)

add_executable(a0509_gui main.cpp pose_math.h)
target_compile_definitions(a0509_gui PRIVATE DRCF_VERSION=${DRCF_VERSION} QT_NO_KEYWORDS)
target_include_directories(a0509_gui PRIVATE ${DRFL_ROOT}/include)
target_link_libraries(a0509_gui PRIVATE
  Qt5::Widgets
  ${DRFL_LIB_DIR}/libDRFL.a
  ${DRFL_LIB_DIR}/libPocoNet.so.80
  ${DRFL_LIB_DIR}/libPocoFoundation.so.80
  pthread dl)
set_target_properties(a0509_gui PROPERTIES BUILD_RPATH "${DRFL_LIB_DIR}")
