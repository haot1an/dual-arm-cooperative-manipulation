cmake_minimum_required(VERSION 3.16)
project(dual_arm_ws VERSION 0.1.0 LANGUAGES CXX)

# ----------------------------------------------------------------------------
# 基本设置
# ----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # 默认 Release：控制周期耗时统计有意义
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(DUAL_ARM_BUILD_TESTS     "Build GTest unit tests"                         ON)
option(DUAL_ARM_BUILD_VIEWER    "Build GLFW viewer (needs glfw3)"                 ON)
option(DUAL_ARM_WITH_OSQP       "Build the optional OsqpEigen compatibility test"  OFF)
option(DUAL_ARM_WITH_PINOCCHIO  "Enable an optional Pinocchio RobotModel backend"  OFF)
option(DUAL_ARM_WERROR          "Treat warnings as errors"                        OFF)

# ----------------------------------------------------------------------------
# 依赖
#   本工程不依赖 ROS。可选依赖 OSQP / Pinocchio 在本机只存在于 /opt/ros/jazzy，
#   因此只有显式打开对应选项时才会去那里查找。
# ----------------------------------------------------------------------------
# MuJoCo 从源码安装在 ~/.local；也可以用 -Dmujoco_DIR=... 指定
find_package(mujoco 3.2 REQUIRED CONFIG HINTS "$ENV{HOME}/.local/lib/cmake/mujoco")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

message(STATUS "MuJoCo  : ${mujoco_VERSION}  (${mujoco_DIR})")
message(STATUS "Eigen3  : ${Eigen3_VERSION}  (${Eigen3_DIR})")
message(STATUS "yaml-cpp: ${yaml-cpp_VERSION}  (${yaml-cpp_DIR})")

if(DUAL_ARM_WITH_OSQP)
  # 注意：本机 /usr/local 下的 libOsqpEigen.so 实际是针对 ROS jazzy 自带的 OSQP 0.6.2 编译的
  # （/usr/local 下另有一份 OSQP 1.x，两者 SONAME 相同）。详见 README “可选依赖”一节。
  find_package(OsqpEigen REQUIRED)
  message(STATUS "OsqpEigen: ${OsqpEigen_DIR}, osqp: ${osqp_DIR}")
endif()

if(DUAL_ARM_WITH_PINOCCHIO)
  # sudo apt install ros-jazzy-pinocchio 后可用；只作为普通 C++ 库使用，不引入任何 ROS 代码
  find_package(pinocchio REQUIRED CONFIG PATHS /opt/ros/jazzy)
  message(STATUS "Pinocchio: ${pinocchio_VERSION} (${pinocchio_DIR})")
endif()

# ----------------------------------------------------------------------------
# 编译警告
# ----------------------------------------------------------------------------
set(DUAL_ARM_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor)
if(DUAL_ARM_WERROR)
  list(APPEND DUAL_ARM_WARNINGS -Werror)
endif()

# ----------------------------------------------------------------------------
# 核心库：dual_arm_core（不含任何 GUI / ROS 代码）
# ----------------------------------------------------------------------------
add_library(dual_arm_core
  src/math_utils.cpp
  src/config.cpp
  src/scene_spec.cpp
  src/mujoco_utils.cpp
  src/sim_env.cpp
  src/mujoco_model.cpp
  src/collision_model.cpp
  src/scene_monitor.cpp
  src/baseline_controllers.cpp
  src/object_trajectory.cpp
  src/coop_kinematics.cpp
  src/coop_controller.cpp
  src/asym_coop_controller.cpp
  src/torque_qp.cpp
  src/qp_coop_controller.cpp
  src/qp_asym_coop_controller.cpp
  src/grasp_alignment.cpp
  src/cbf_reference_filter.cpp
  src/small_qp.cpp
  src/sparse_qp.cpp
  src/path_deformation.cpp
  src/closed_chain_ik.cpp
  src/trajectory_planner.cpp
  src/trajectory_optimizer.cpp
  src/logger.cpp
)
target_include_directories(dual_arm_core PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(dual_arm_core PUBLIC mujoco::mujoco Eigen3::Eigen PRIVATE yaml-cpp::yaml-cpp)
target_compile_options(dual_arm_core PRIVATE ${DUAL_ARM_WARNINGS})
# 工程根目录：config 中的相对路径（models/、logs/）以它为基准
target_compile_definitions(dual_arm_core PUBLIC DUAL_ARM_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
if(DUAL_ARM_WITH_OSQP)
  target_link_libraries(dual_arm_core PUBLIC OsqpEigen::OsqpEigen)
  target_compile_definitions(dual_arm_core PUBLIC DUAL_ARM_HAS_OSQP=1)
endif()
if(DUAL_ARM_WITH_PINOCCHIO)
  target_link_libraries(dual_arm_core PUBLIC pinocchio::pinocchio)
  target_compile_definitions(dual_arm_core PUBLIC DUAL_ARM_HAS_PINOCCHIO=1)
endif()

# ----------------------------------------------------------------------------
# 可视化：dual_arm_viewer（GLFW + MuJoCo 自带渲染）
# ----------------------------------------------------------------------------
if(DUAL_ARM_BUILD_VIEWER)
  find_package(glfw3 3.3 REQUIRED)
  find_package(ZLIB REQUIRED)  # 截图存 PNG
  add_library(dual_arm_viewer src/viewer.cpp)
  target_link_libraries(dual_arm_viewer PUBLIC dual_arm_core PRIVATE glfw ZLIB::ZLIB)
  target_compile_options(dual_arm_viewer PRIVATE ${DUAL_ARM_WARNINGS})
endif()

# ----------------------------------------------------------------------------
# 可执行程序
# ----------------------------------------------------------------------------
add_executable(run_sim apps/run_sim.cpp)
target_link_libraries(run_sim PRIVATE dual_arm_core)
target_compile_options(run_sim PRIVATE ${DUAL_ARM_WARNINGS})
if(DUAL_ARM_BUILD_VIEWER)
  target_link_libraries(run_sim PRIVATE dual_arm_viewer)
  target_compile_definitions(run_sim PRIVATE DUAL_ARM_HAS_VIEWER=1)
endif()
# 把 run_sim 放在 build/ 根目录，方便 ./build/run_sim
set_target_properties(run_sim PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

# 独立接触夹取实验：四个可动手指、摩擦接触、物体无 weld。
add_executable(run_contact_grasp apps/run_contact_grasp.cpp)
target_link_libraries(run_contact_grasp PRIVATE dual_arm_core)
target_compile_options(run_contact_grasp PRIVATE ${DUAL_ARM_WARNINGS})
if(DUAL_ARM_BUILD_VIEWER)
  target_link_libraries(run_contact_grasp PRIVATE dual_arm_viewer)
  target_compile_definitions(run_contact_grasp PRIVATE DUAL_ARM_HAS_VIEWER=1)
endif()
set_target_properties(run_contact_grasp PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

# 离线工具（tools/*.py）通过 ctypes 调用的 MuJoCo 小接口：与仿真用同一个 MuJoCo 版本
add_library(dual_arm_mjshim SHARED tools/mjshim.cpp)
target_link_libraries(dual_arm_mjshim PRIVATE mujoco::mujoco)
target_compile_options(dual_arm_mjshim PRIVATE ${DUAL_ARM_WARNINGS})
set_target_properties(dual_arm_mjshim PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

# ----------------------------------------------------------------------------
# 测试
# ----------------------------------------------------------------------------
if(DUAL_ARM_BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()
  include(GoogleTest)

  add_executable(dual_arm_tests
    tests/test_math_utils.cpp
    tests/test_jacobian.cpp
    tests/test_gravity_comp.cpp
    tests/test_sensors_weld.cpp
    tests/test_calibration.cpp
    tests/test_scene_geometry.cpp
    tests/test_coop_template.cpp
    tests/test_collision_model.cpp
    tests/test_scenes.cpp
    tests/test_trajectory.cpp
    tests/test_torque_qp.cpp
    tests/test_cbf_reference_filter.cpp
    tests/test_trajectory_planner.cpp
  )
  target_link_libraries(dual_arm_tests PRIVATE dual_arm_core GTest::gtest_main)
  target_compile_options(dual_arm_tests PRIVATE ${DUAL_ARM_WARNINGS})
  gtest_discover_tests(dual_arm_tests DISCOVERY_TIMEOUT 60)

  # 单独的可执行文件：替换了 malloc 来统计控制环中的堆分配（含 operator new、Eigen、MuJoCo）
  add_executable(test_no_alloc tests/test_no_alloc.cpp)
  target_link_libraries(test_no_alloc PRIVATE dual_arm_core GTest::gtest_main)
  target_compile_options(test_no_alloc PRIVATE ${DUAL_ARM_WARNINGS})
  gtest_discover_tests(test_no_alloc DISCOVERY_TIMEOUT 60)

  add_test(NAME ContactGrasp.PhysicalLift COMMAND run_contact_grasp --headless --check)
  add_test(NAME ContactGrasp.SlotInsertion
    COMMAND run_sim --scene slot --controller qp_coop --contact-grasp --check-contact --headless --no-log --duration 19)
  add_test(NAME ContactGrasp.SlotAvoidance
    COMMAND run_sim --scene slot_avoid --controller qp_coop --contact-grasp --check-contact --headless --no-log
            --duration 24 --set "disturbances=[]")
  add_test(NAME ContactGrasp.SlotGatePlanned
    COMMAND run_sim --scene slot_gate --controller qp_coop --contact-grasp --check-contact --headless --no-log
            --duration 24 --set "disturbances=[]")
  add_test(NAME ContactGrasp.SlotGateModelErrorWithCbf
    COMMAND run_sim --scene slot_gate --controller qp_coop --contact-grasp --check-contact --headless --no-log
            --duration 16 --set "disturbances=[]" --set planner.model_error_body=gate
            --set "planner.model_error_offset=[0,0,0.03]")
  add_test(NAME ContactGrasp.ScrewAssembly
    COMMAND run_sim --scene assembly --controller qp_asym_coop --contact-grasp --check-contact --headless --no-log --duration 21)

  if(DUAL_ARM_WITH_OSQP)
    add_executable(test_osqp_smoke tests/test_osqp_smoke.cpp)
    target_link_libraries(test_osqp_smoke PRIVATE dual_arm_core GTest::gtest_main)
    gtest_discover_tests(test_osqp_smoke)
  endif()
endif()
