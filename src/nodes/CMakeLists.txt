add_executable(semantic_mapper_node
  semantic_mapper_node.cpp
)

target_link_libraries(semantic_mapper_node
  semantic_mapper_library
  map_evaluator_library
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(pose_broadcaster_node
  pose_broadcaster_node.cpp
)

target_link_libraries(pose_broadcaster_node
  ${catkin_LIBRARIES}
)

add_executable(map_evaluator_node
  map_evaluator_node.cpp
)

target_link_libraries(map_evaluator_node
  map_evaluator_library
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

