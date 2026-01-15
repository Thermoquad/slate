# SPDX-License-Identifier: GPL-2.0-or-later

cmake_minimum_required(VERSION 3.20.0)

# Add fusain module to EXTRA_ZEPHYR_MODULES
list(APPEND EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/fusain)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(slate)

# Slate sources
FILE(GLOB_RECURSE app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
target_include_directories(app PRIVATE include)
