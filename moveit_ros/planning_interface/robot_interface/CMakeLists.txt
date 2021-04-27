set(MOVEIT_LIB_NAME moveit_robot_interface)

add_library(${MOVEIT_LIB_NAME} src/wrap_python_robot_interface.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES}
  moveit_common_planning_interface_objects moveit_py_bindings_tools visualization_msgs
)

ament_target_dependencies(${MOVEIT_LIB_NAME}
  visualization_msgs
)


# set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
# set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES OUTPUT_NAME _moveit_robot_interface PREFIX "")
# set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY "bin")
# if(WIN32)
#   set_target_properties(${MOVEIT_LIB_NAME}_python PROPERTIES SUFFIX .pyd)
# endif()

# install(TARGETS ${MOVEIT_LIB_NAME}_python
#   EXPORT ${MOVEIT_LIB_NAME}_python
#   DESTINATION bin
# )
