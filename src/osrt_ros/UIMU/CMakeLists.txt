set(DEPENDENCY_LIBS 
  ${catkin_LIBRARIES}
  #${OpenSim_LIBRARIES}
  #${opensimrt_LIBRARIES}
  osrtRosUIMU
)

add_executable(OnlineIK nodes/OnlineIK.cpp)

add_dependencies(OnlineIK ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(OnlineIK
  ${DEPENDENCY_LIBS} osrtRosPipeline
)

add_executable(wth_node nodes/what_the_heading.cpp)

add_dependencies(wth_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(wth_node
${DEPENDENCY_LIBS}
)


