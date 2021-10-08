set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(read_rosbag_data read_rosbag_data.cpp)
target_link_libraries(read_rosbag_data rostools ${Boost_LIBRARIES} ${BZIP2_LIBRARIES} ${lz4_LIBRARIES})

add_executable(read_rosbag_topic read_rosbag_topic.cpp)
target_link_libraries(read_rosbag_topic rostools ${Boost_LIBRARIES} ${BZIP2_LIBRARIES} ${lz4_LIBRARIES})