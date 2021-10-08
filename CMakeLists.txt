cmake_minimum_required(VERSION 2.8)
project(easyrosbag)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")

## Compile as C++14
set (CMAKE_BUILD_TYPE "Release")
# set (CMAKE_BUILD_TYPE "Debug")
set (CMAKE_CXX_FLAGS "-fPIC --std=c++14 -pthread")
set (CMAKE_EXPORT_COMPILE_COMMANDS)
 
IF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86")
    message("-- Current Sys is Linux x86")
    include_directories(${PROJECT_SOURCE_DIR}/tools/OpenSourceLib/x86_64/libmosquito_x86_64/include)
    link_directories(${PROJECT_SOURCE_DIR}/tools/OpenSourceLib/x86_64/libmosquito_x86_64/lib)
ELSEIF(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message("-- Current Sys is Linux arm")
    include_directories(${PROJECT_SOURCE_DIR}/tools/OpenSourceLib/arm_64/libmosquito_arm64/include)
    link_directories(${PROJECT_SOURCE_DIR}/tools/OpenSourceLib/arm_64/libmosquito_arm64/lib)
ELSE()
    message("-- Current Sys is unsupport")
ENDIF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_subdirectory(${PROJECT_SOURCE_DIR}/src/rostools)

include_directories(
    include
    include/easyrosbag
    include/easyrosbag/3rdParty
    include/easyrosbag/3rdParty/rapidjson/include/
    include/easyrosbag/communication
    include/easyrosbag/utils
    include/easyrosbag/visualization
)

include_directories( 
    ${PROJECT_SOURCE_DIR}/src/rostools
    ${PROJECT_SOURCE_DIR}/src/rostools/include
    ${PROJECT_SOURCE_DIR}/src/rostools/console_bridge/include
    ${PROJECT_SOURCE_DIR}/src/rostools/roscpp_core/cpp_common/include
    ${PROJECT_SOURCE_DIR}/src/rostools/roscpp_core/rostime/include
    ${PROJECT_SOURCE_DIR}/src/rostools/roscpp_core/roscpp_serialization/include
    ${PROJECT_SOURCE_DIR}/src/rostools/roscpp_core/roscpp_traits/include
    ${PROJECT_SOURCE_DIR}/src/rostools/ros_comm/utilities/roslz4/include
    ${PROJECT_SOURCE_DIR}/src/rostools/ros_comm/tools/rosbag_storage/include
)
link_directories(${PROJECT_SOURCE_DIR}/src/rostools/lib)

# set bin folder
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

file(GLOB SOURCE_LISTS 
    ${PROJECT_SOURCE_DIR}/src/communication/*.cpp
    ${PROJECT_SOURCE_DIR}/src/data_exchange/*.cpp
    ${PROJECT_SOURCE_DIR}/src/visualization/*.cpp
)

add_executable(rosbag_play ${PROJECT_SOURCE_DIR}/examples/rosbag_play.cpp ${SOURCE_LISTS})
target_link_libraries(rosbag_play ${EIGEN_LIBRARIES} ${Pangolin_LIBRARIES} rostools mosquitto mosquittopp)

add_executable(rosbag_viz ${PROJECT_SOURCE_DIR}/examples/rosbag_viz.cpp ${SOURCE_LISTS})
target_link_libraries(rosbag_viz ${EIGEN_LIBRARIES} ${Pangolin_LIBRARIES} rostools mosquitto mosquittopp)