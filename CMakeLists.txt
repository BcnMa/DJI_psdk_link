cmake_minimum_required(VERSION 3.10)
project(plink_demo)

set(CMAKE_CXX_STANDARD 11)

# 设置构建类型为 Debug
set(CMAKE_BUILD_TYPE Debug)
# 可选：自定义调试标志
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

include_directories(./hal)
include_directories(./link)

add_executable(plink_demo main.cpp link/p_link_cmp.cpp link/p_link.cpp hal/hal_uart.c)


