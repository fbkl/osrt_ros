cmake_minimum_required(VERSION 3.0.2)
project(osrt_ros)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

add_compile_options(-std=c++17 -O3 -g -Wall -pedantic -Wno-undef)
#add_compile_options(-std=c++17 -O0 -g)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -g")

find_package(OpenSim REQUIRED PATHS "${OPENSIM_HOME}"
		simbody
	)
#find_package(SimTK REQUIRED)
find_package(TinyXML2 REQUIRED)

find_package(Eigen3 3.3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	dynamic_reconfigure
	message_runtime
	message_generation
	opensimrt
	opensimrt_msgs
	opensimrt_bridge
	roscpp
	rosdoc_lite
	sensor_msgs
	tf
	tf2
	tf2_ros
	)
#find_package(Simbody REQUIRED PATHS "/opt/dependencies/opensim-core/dependencies/simbody/lib/cmake/simbody/")

# Define the target for Doxygen documentation
message(NOTICE "Building doxygen?-----------------------------------------------------------------")
# consider adding tags to link to docs from each package.
add_custom_target(${PROJECT_NAME}_docs ALL
  COMMAND rosdoc_lite -o docs ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating Doxygen documentation"
)

generate_dynamic_reconfigure_options(
	cfg/UIMU.cfg
	cfg/heading.cfg
	cfg/delay.cfg
)

add_service_files(
	DIRECTORY srv
	FILES
	Float.srv
	)
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   )

find_package(opensimrt REQUIRED COMPONENTS
	Common
	Realtime)

find_package(opensimrt_bridge REQUIRED COMPONENTS
	conversions
	)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS 
	message_runtime
	opensimrt_msgs 
	opensimrt_bridge
	roscpp
	sensor_msgs
	tf2
	tf2_ros
	LIBRARIES osrtRosUIMU osrtRosPipeline
	)

message(NOTICE "Eigen3")
message(STATUS
	${EIGEN3_INCLUDE_DIRS}

	)
message(NOTICE "OpenSim")
message(STATUS
	${OpenSim_INCLUDE_DIRS}

	)
message(NOTICE "simbody")
message(STATUS
	${simbody_INCLUDE_DIRS}

	)

message(NOTICE "catkin")
foreach(dir ${catkin_INCLUDE_DIRS})
	message(STATUS
		"catkin_dir=${dir}"
		)
endforeach()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	)

##PIPELINE ############################################################

file (GLOB Pipeline_SOURCES CONFIGURE_DEPENDS "src/osrt_ros/Pipeline/*.cpp" "src/osrt_ros/events.cpp" "src/osrt_ros/parameters.cpp")
file (GLOB Pipeline_HEADERS CONFIGURE_DEPENDS "include/Pipeline/*.h")

set(target osrtRosPipeline)

#include_directories("/opt/dependencies/opensim-core/include/OpenSim/"
#	"/opt/dependencies/opensim-core/include/simbody/"
#	)

add_library(osrtRosPipeline ${Pipeline_HEADERS} ${Pipeline_SOURCES} )

target_link_libraries(osrtRosPipeline ${catkin_LIBRARIES} ${OpenSim_LIBRARIES})

add_dependencies(osrtRosPipeline ${catkin_EXPORTED_TARGETS})

add_subdirectory(src/osrt_ros/Pipeline)

###/PIPELINE############################################################

###VISUALIZER############################################################

file (GLOB Visualizers_SOURCES CONFIGURE_DEPENDS "src/osrt_ros/Visualizers/*.cpp" "src/osrt_ros/events.cpp" ) # do i need events?
file (GLOB Visualizers_HEADERS CONFIGURE_DEPENDS "include/Visualizers/*.h")

set(target osrtRosVisualizers)

add_library(osrtRosVisualizers ${Visualizers_HEADERS} ${Visualizers_SOURCES} )

target_link_libraries(osrtRosVisualizers ${catkin_LIBRARIES} ${OpenSim_LIBRARIES})

add_dependencies(osrtRosVisualizers ${catkin_EXPORTED_TARGETS})

add_subdirectory(src/osrt_ros/Visualizers)
###\VISUALIZER############################################################

###UIMU############################################################
file (GLOB UIMU_SOURCES CONFIGURE_DEPENDS "src/osrt_ros/UIMU/*.cpp" "src/osrt_ros/events.cpp" )
file (GLOB UIMU_HEADERS CONFIGURE_DEPENDS "include/UIMU/*.h")

add_subdirectory(src/osrt_ros/UIMU)

set(target osrtRosUIMU)

add_library(osrtRosUIMU ${UIMU_HEADERS} ${UIMU_SOURCES} )

add_dependencies(osrtRosUIMU ${catkin_EXPORTED_TARGETS})

target_link_libraries(osrtRosUIMU ${catkin_LIBRARIES} ${OpenSim_LIBRARIES} ${opensimrt_LIBRARIES} Eigen3::Eigen)

###\UIMU############################################################

add_executable(custom_joints_node src/osrt_ros/custom_joints_node.cpp)
add_dependencies(custom_joints_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_joints_node
  ${catkin_LIBRARIES}
  ${OpenSim_LIBRARIES}
  ${opensimrt_LIBRARIES}
  osrtRosPipeline
)

add_executable(osim_to_simple_urdf src/osrt_ros/osim_to_simple_urdf.cpp)
add_dependencies(osim_to_simple_urdf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(osim_to_simple_urdf
  ${catkin_LIBRARIES}
 tinyxml2
  ${OpenSim_LIBRARIES}
  ${opensimrt_LIBRARIES}
  osrtRosPipeline
)


add_executable(external_average_quaternion_node src/osrt_ros/external_average_pose.cpp)
add_dependencies(external_average_quaternion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(external_average_quaternion_node
  ${catkin_LIBRARIES}
)


add_executable(o_node src/osrt_ros/osimtfpublisher.cpp)
add_dependencies(o_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(o_node
  ${catkin_LIBRARIES}
  ${OpenSim_LIBRARIES}
  ${opensimrt_LIBRARIES}
)


