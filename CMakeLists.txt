cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 20)

project(robot-model)

# Used to find GTest on CAEN
list(APPEND CMAKE_PREFIX_PATH "~/.local")

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)

if (DEFINED GUI)
find_package(Open3D REQUIRED)
endif()

# Used to compile on Autograder (possibly unnecessary ...)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT DEFINED ARCH)
    # If we do not pre-define our architecture, run this command to infer it
    EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCH)
    # Normalize name
    if(ARCH STREQUAL "arm64")
        set(ARCH "aarch64")
    endif()
endif()

if(NOT DEFINED OS)
    # If we do not pre-define our operating system, infer it
    if(APPLE)
        set(OS "macos")
    elseif(UNIX)
        set(OS "linux")
    else()
        message(FATAL_ERROR "Unsupported OS.")
    endif()
endif()

message(STATUS "Architecture: ${ARCH}")
message(STATUS "Operating System: ${OS}")

# Compile the Project 4 library
add_library(project4 src/rix/robot/joint.cpp
    src/rix/robot/link.cpp
    src/rix/robot/robot_model.cpp
    src/rix/robot/eigen_util.cpp
    src/rix/tf/frame_graph.cpp
    src/rix/tf/frame.cpp
    src/rix/tf/transform_broadcaster.cpp
    src/rix/tf/transform_buffer.cpp
    src/rix/tf/transform_listener.cpp
)
target_include_directories(project4 PRIVATE include/)
target_link_libraries(project4 PUBLIC Threads::Threads Eigen3::Eigen nlohmann_json::nlohmann_json)

# Link against the proper Project 2 library
target_link_libraries(project4 PUBLIC
    ${CMAKE_SOURCE_DIR}/lib/${OS}-${ARCH}/libproject3.a
    ${CMAKE_SOURCE_DIR}/lib/${OS}-${ARCH}/libproject2.a
    ${CMAKE_SOURCE_DIR}/lib/${OS}-${ARCH}/libproject1.a
)

# Add robot_model test
add_executable(joint_test tests/joint.cpp)
target_include_directories(joint_test PRIVATE include/)
target_link_libraries(joint_test project4 GTest::Main)

add_executable(robot_model_simple_bot_test tests/robot_model_simple_bot.cpp)
target_include_directories(robot_model_simple_bot_test PRIVATE include/)
target_link_libraries(robot_model_simple_bot_test project4 GTest::Main)

add_executable(robot_model_rx200_test tests/robot_model_rx200.cpp)
target_include_directories(robot_model_rx200_test PRIVATE include/)
target_link_libraries(robot_model_rx200_test project4 GTest::Main)

add_executable(frame_graph_test tests/frame_graph.cpp)
target_include_directories(frame_graph_test PRIVATE include/)
target_link_libraries(frame_graph_test project4 GTest::Main)

add_executable(transform_buffer_test tests/transform_buffer.cpp)
target_include_directories(transform_buffer_test PRIVATE include/)
target_link_libraries(transform_buffer_test project4 GTest::Main)

add_executable(random_setpoint_publisher
    src/random_setpoint_publisher/random_setpoint_publisher.cpp
    src/random_setpoint_publisher/main.cpp
)
target_include_directories(random_setpoint_publisher PRIVATE include/)
target_link_libraries(random_setpoint_publisher project4)

add_executable(joint_state_publisher
    src/joint_state_publisher/joint_state_publisher.cpp
    src/joint_state_publisher/main.cpp
)
target_include_directories(joint_state_publisher PRIVATE include/)
target_link_libraries(joint_state_publisher project4)

add_executable(joint_state_publisher_test
    src/joint_state_publisher/joint_state_publisher.cpp
    tests/joint_state_publisher.cpp
)
target_include_directories(joint_state_publisher_test PRIVATE include/)
target_link_libraries(joint_state_publisher_test project4 GTest::gtest_main GTest::gmock)

add_executable(robot_state_publisher
src/robot_state_publisher/robot_state_publisher.cpp
src/robot_state_publisher/main.cpp
)
target_include_directories(robot_state_publisher PRIVATE include/)
target_link_libraries(robot_state_publisher project4)

add_executable(robot_state_publisher_test
    src/robot_state_publisher/robot_state_publisher.cpp
    tests/robot_state_publisher.cpp
)
target_include_directories(robot_state_publisher_test PRIVATE include/)
target_link_libraries(robot_state_publisher_test project4 GTest::gtest_main GTest::gmock)
