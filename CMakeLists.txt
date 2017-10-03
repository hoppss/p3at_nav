cmake_minimum_required(VERSION 2.8.3)
project(p3at_nav)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  cv_bridge
  roscpp
  rospy
  std_msgs
  genmsg
  tf 
  turtlesim
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_message_files(
	FILES
	Num.msg
)

add_service_files(
	FILES
	AddTwoInts.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)



add_executable(samson1 src/samson1.cpp)
target_link_libraries(samson1
	${catkin_LIBRARIES}
)
add_dependencies(samson1 RosAria)

add_executable(samson2 src/samson2.cpp)
target_link_libraries(samson2
	${catkin_LIBRARIES}
)
add_dependencies(samson2 RosAria)


add_executable(atratores src/atratores.cpp)
target_link_libraries(atratores
	${catkin_LIBRARIES}
)
add_dependencies(atratores RosAria)

add_executable(robot_atrator src/robot_atrator.cpp)
target_link_libraries(robot_atrator
	${catkin_LIBRARIES}
)
add_dependencies(robot_atrator RosAria)

add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster
	${catkin_LIBRARIES}
)


add_executable(particle_filter src/particle_filter.cpp)
target_link_libraries( particle_filter
	${catkin_LIBRARIES}
)

add_dependencies(particle_filter RosAria)

add_executable(consenso src/consenso.cpp)
target_link_libraries(consenso
	${catkin_LIBRARIES}
)

add_dependencies(consenso RosAria)

add_executable(consenso1 src/consenso1.cpp)
target_link_libraries(consenso1
	${catkin_LIBRARIES}
)

add_dependencies(consenso1 RosAria)

add_executable(consenso2 src/consenso2.cpp)
target_link_libraries(consenso2
	${catkin_LIBRARIES}
)

add_dependencies(consenso2 RosAria)

add_executable(consenso3 src/consenso3.cpp)
target_link_libraries(consenso3
	${catkin_LIBRARIES}
)

add_dependencies(consenso3 RosAria)

add_executable(qlearning src/qlearning.cpp)
target_link_libraries(qlearning
	${catkin_LIBRARIES}
)

add_dependencies(qlearning RosAria)

add_executable(rl src/rl.cpp)
target_link_libraries(rl
	${catkin_LIBRARIES}
)

add_dependencies(rl RosAria)

add_executable(cv_bridge2 src/cv_bridge2.cpp)
target_link_libraries(cv_bridge2
	${catkin_LIBRARIES}
)

add_dependencies(cv_bridge2 RosAria)

add_executable(reward src/reward.cpp)
target_link_libraries(reward
	${catkin_LIBRARIES}
)

add_dependencies(reward RosAria)

