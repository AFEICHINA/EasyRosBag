cmake_minimum_required(VERSION 3.0)
project(rostools)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
 
IF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86")
    # message("Current Sys is Linux x86")
    find_package(Boost REQUIRED COMPONENTS date_time filesystem program_options regex)
ELSEIF(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # message("Current Sys is Linux arm")
    find_package(Boost REQUIRED)
ELSE()
    message("Current Sys is unsupport")
ENDIF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86")

find_package(BZip2 REQUIRED)
find_library(lz4_LIBRARIES NAMES lz4)

file(GLOB CONSOLE_BRIDGE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/console_bridge/src/*.cpp")
file(GLOB CPP_COMMON_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/roscpp_core/cpp_common/src/*.cpp")
file(GLOB ROSCPP_SERIALIZATION_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/roscpp_core/roscpp_serialization/src/*.cpp")
file(GLOB ROSTIME_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/roscpp_core/rostime/src/*.cpp")
file(GLOB ROSBAG_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/ros_comm/tools/rosbag_storage/src/*.cpp")
file(GLOB ROSLZ4_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/ros_comm/utilities/roslz4/src/[a-z]*.c")

include_directories( 
    include
    console_bridge/include
    roscpp_core/cpp_common/include
    roscpp_core/rostime/include
    roscpp_core/roscpp_serialization/include
    roscpp_core/roscpp_traits/include
    ros_comm/utilities/roslz4/include
    ros_comm/tools/rosbag_storage/include
    )

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(rostools STATIC
        ${ROSBAG_SRCS} 
        ${ROSLZ4_SRCS} 
        ${ROSTIME_SRCS} 
        ${CPP_COMMON_SRCS} 
        ${ROSCPP_SERIALIZATION_SRCS} 
        ${CONSOLE_BRIDGE_SRCS}
        )

target_link_libraries(rostools PUBLIC ${Boost_LIBRARIES} ${BZIP2_LIBRARIES} ${lz4_LIBRARIES})

add_subdirectory(examples)