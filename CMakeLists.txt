cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_recorder)

find_package(catkin REQUIRED COMPONENTS point_cloud_transport sensor_msgs rosbag)

catkin_package(CATKIN_DEPENDS point_cloud_transport sensor_msgs rosbag)

include_directories(include ${catkin_INCLUDE_DIRS})

# publisher
add_executable(recorder src/point_cloud_recorder.cpp)
add_dependencies(recorder ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(recorder ${catkin_LIBRARIES})


# subscriber
add_executable(decoder src/point_cloud_decorder.cpp)
add_dependencies(decoder ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(decoder ${catkin_LIBRARIES})


# Mark executables and/or libraries for installation
#install(TARGETS recorder decoder
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

