add_library(semantic_mapper_node_library SHARED
  semantic_mapper_node.h semantic_mapper_node.cpp
)

target_link_libraries(semantic_mapper_node_library
  object_detector_library
  semantic_mapper_library
  ${catkin_LIBRARIES}
)

#add_executable(lucrezio_semantic_mapper_node
#  lucrezio_semantic_mapper_node.cpp
#)

#target_link_libraries(lucrezio_semantic_mapper_node
#  semantic_mapper_node_library
#  ${catkin_LIBRARIES}
#)
