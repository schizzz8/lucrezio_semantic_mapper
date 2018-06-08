add_executable(lucrezio_semantic_mapper_node lucrezio_semantic_mapper_node.cpp)

target_link_libraries(lucrezio_semantic_mapper_node
  lucrezio_semantic_mapping_library
  ${catkin_LIBRARIES}
)

