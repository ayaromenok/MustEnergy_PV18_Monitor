cmake_minimum_required(VERSION 3.13)
project(pv18_monitor C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

# libpaho-mqtt (MQTTAsync C client). Prefer pkg-config, fall back to
# find_library (Debian/Ubuntu ship libpaho-mqtt3c without a .pc file).
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(PAHO QUIET libpaho-mqtt)
endif()

if(PAHO_FOUND)
    set(PAHO_LIBRARIES ${PAHO_LIBRARIES})
    set(PAHO_INCLUDE_DIRS ${PAHO_INCLUDE_DIRS})
    set(PAHO_LIBRARY_DIRS ${PAHO_LIBRARY_DIRS})
else()
    find_path(PAHO_INCLUDE_DIR MQTTAsync.h)
    # async API: paho-mqtt3a (Debian/Ubuntu split it out of paho-mqtt3c)
    find_library(PAHO_LIBRARY NAMES paho-mqtt3a paho-mqtt3c paho-mqtt3)
    if(NOT PAHO_INCLUDE_DIR OR NOT PAHO_LIBRARY)
        message(FATAL_ERROR "libpaho-mqtt not found "
                "(install libpaho-mqtt-dev)")
    endif()
    set(PAHO_LIBRARIES ${PAHO_LIBRARY})
endif()

find_package(Threads REQUIRED)

add_executable(pv18_monitor pv18_monitor.c)
target_include_directories(pv18_monitor PRIVATE ${PAHO_INCLUDE_DIRS})
target_link_directories(pv18_monitor PRIVATE ${PAHO_LIBRARY_DIRS})
target_link_libraries(pv18_monitor PRIVATE ${PAHO_LIBRARIES} Threads::Threads m)
