set(DEPENDENCY_LIBS 
  ${catkin_LIBRARIES}
  #${OpenSim_LIBRARIES}
  #${opensimrt_LIBRARIES}
  osrtRosUIMU
)

add_executable(OnlineLowerLimb nodes/OnlineLowerLimbUIMUIK.cpp)

add_dependencies(OnlineLowerLimb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(OnlineLowerLimb
  ${DEPENDENCY_LIBS} osrtRosPipeline
)
# OnlineUpperLimb removed 2026-08-25. It differed from OnlineLowerLimb only by
# registering Schutte1993Muscle_Deprecated instead of Thelen2003Muscle -- and neither
# type appears in any model we use, while the type that DOES appear (MOBL's 50x
# Millard2012EquilibriumMuscle) is auto-registered by osimActuatorsInstantiator. IK
# never touches actuators anyway. See first_paperino.md.

add_executable(wth_node nodes/what_the_heading.cpp)

add_dependencies(wth_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(wth_node
${DEPENDENCY_LIBS}
)


