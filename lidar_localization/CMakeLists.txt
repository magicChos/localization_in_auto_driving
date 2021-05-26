cmake_minimum_required(VERSION 2.8.3)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  message_generation 
  std_srvs
)

add_service_files(
   FILES
   saveMap.srv
   # optimizeMap.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

set(ALL_TARGET_LIBRARIES "")
find_package(Gflags)
find_package(Glog)

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)
# include(cmake/g2o.cmake)

include_directories(${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/build/devel/include")
message("------------------------------------------")
message(${catkin_INCLUDE_DIRS})

include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "src/front_end/*.cpp" "src/models/*.cpp" "src/publisher/*.cpp" "src/sensor_data/*.cpp" "src/subscriber/*.cpp" "src/tf_listener/*.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
file(GLOB_RECURSE BUILD_SRCS "build/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})
list(REMOVE_ITEM ALL_SRCS ${BUILD_SRCS})
list(APPEND ALL_TARGET_LIBRARIES ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})

add_executable(test_frame_node src/test_frame_node.cpp ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})


add_executable(front_end_node src/front_end_node.cpp ${ALL_SRCS})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})
