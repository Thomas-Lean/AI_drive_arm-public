cmake_minimum_required(VERSION 3.10)
project(d1t_arm_ctrl CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-Wall -Wextra)

# DDS：臂上 Cyclone DDS C++ 0.10.2（任务书 §1.1）。生成代码由 T2 提交到 idl/generated/。
# 默认 AUTO：找得到 CycloneDDS-CXX 且 idl/generated/ 存在就启用，否则只编核心 + 仿真。
set(D1T_WITH_DDS AUTO CACHE STRING "ON / OFF / AUTO")

find_package(Threads REQUIRED)

add_library(d1t_core STATIC
  src/bus/frame.cpp
  src/bus/bus.cpp
  src/bus/posix_serial.cpp
  src/sim/fake_serial.cpp
  src/safety/safety.cpp
  src/loop/mailbox.cpp
  src/loop/controller.cpp
  src/sim/fake_client.cpp
)
target_include_directories(d1t_core PUBLIC src)
target_link_libraries(d1t_core PUBLIC Threads::Threads)

# ---- DDS ----
set(_dds_enabled OFF)
file(GLOB _gen_srcs ${CMAKE_CURRENT_SOURCE_DIR}/idl/generated/*.cpp)
set(_dds_manual OFF)
if(NOT D1T_WITH_DDS STREQUAL "OFF")
  find_package(CycloneDDS-CXX QUIET)
  if(CycloneDDS-CXX_FOUND AND _gen_srcs)
    set(_dds_enabled ON)
  elseif(_gen_srcs AND EXISTS /usr/local/include/ddscxx/dds/dds.hpp)
    # 臂上：/usr/local/include/ddscxx + /usr/local/lib/libddscxx.so（F62），找不到包配置时按该路径直接引
    set(_dds_enabled ON)
    set(_dds_manual ON)
  elseif(D1T_WITH_DDS STREQUAL "ON")
    message(FATAL_ERROR "D1T_WITH_DDS=ON 但缺 CycloneDDS-CXX 或 idl/generated/*.cpp")
  endif()
endif()
message(STATUS "D1T DDS: ${_dds_enabled} (manual paths: ${_dds_manual})")

add_executable(d1t_arm_ctrl src/main.cpp)
target_link_libraries(d1t_arm_ctrl PRIVATE d1t_core)
if(_dds_enabled)
  target_sources(d1t_arm_ctrl PRIVATE src/dds_io/dds_io_cyclone.cpp ${_gen_srcs})
  target_include_directories(d1t_arm_ctrl PRIVATE idl/generated)
  if(_dds_manual)
    # 手动路径：include ddscxx + iceoryx/v2.0.2，链接 ddsc ddscxx rt pthread（F62）
    target_include_directories(d1t_arm_ctrl PRIVATE /usr/local/include/ddscxx /usr/local/include)
    if(EXISTS /usr/local/include/iceoryx/v2.0.2)
      target_include_directories(d1t_arm_ctrl PRIVATE /usr/local/include/iceoryx/v2.0.2)
    endif()
    target_link_libraries(d1t_arm_ctrl PRIVATE ddscxx ddsc rt pthread)
  else()
    target_link_libraries(d1t_arm_ctrl PRIVATE CycloneDDS-CXX::ddscxx)
  endif()
  target_compile_definitions(d1t_arm_ctrl PRIVATE D1T_WITH_DDS=1)
endif()

# ---- 测试（Mac 无硬件可跑） ----
enable_testing()
foreach(t test_frame test_safety test_sim test_regress)
  add_executable(${t} tests/${t}.cpp)
  target_link_libraries(${t} PRIVATE d1t_core)
  target_include_directories(${t} PRIVATE tests)
endforeach()
add_test(NAME frame COMMAND test_frame)
add_test(NAME safety COMMAND test_safety)
add_test(NAME sim_quick COMMAND test_sim --quick)
add_test(NAME regress COMMAND test_regress)  # 历史缺陷复现场景（永久回归）
