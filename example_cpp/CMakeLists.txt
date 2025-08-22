cmake_minimum_required(VERSION 3.10)
project(HaitaiMotorControl VERSION 1.0)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加Linux定义
add_definitions(-D_GNU_SOURCE)

# 查找线程库
find_package(Threads REQUIRED)

# 包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 源文件设置 - 注意源文件在include文件夹中
set(SOURCES
    include/HaitaiMotorController.cpp
)

# 创建库
add_library(haitai_motor_controller STATIC ${SOURCES})

# 链接线程库
target_link_libraries(haitai_motor_controller
    Threads::Threads
    m
)

# 示例程序
add_executable(sine_plot_example src/example.cpp)

# 链接示例程序
target_link_libraries(sine_plot_example
    haitai_motor_controller
    m
)

# 设置输出目录
set_target_properties(sine_plot_example PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# 打印配置信息
message(STATUS "")
message(STATUS "======= 海泰电机控制库配置 =======")
message(STATUS "构建类型: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++编译器: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "安装路径: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "==================================")
message(STATUS "")
