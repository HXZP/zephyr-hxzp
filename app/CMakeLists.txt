cmake_minimum_required(VERSION 3.20)

# check BOARD variable
if(NOT BOARD)
    set(BOARD hxzp_main)
    message("BOARD not defined, use default value: ${BOARD}")
else()
    message("Use BOARD: ${BOARD}")
endif()


# import zephyr library
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# define cmake project
project(duck)

target_sources(app PRIVATE
    ./src/main.cpp
)

# 将上层模块目录添加到Zephyr的额外模块搜索路径中**
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/..  
)
