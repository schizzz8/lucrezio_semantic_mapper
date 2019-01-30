add_executable(semantic_mapper_node
  semantic_mapper_node.cpp
)

target_link_libraries(semantic_mapper_node
  semantic_mapper_library
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)
