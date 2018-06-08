add_library(lucrezio_semantic_mapping_library SHARED
  object.h object.cpp
  semantic_map.h semantic_map.cpp
  semantic_mapper.h semantic_mapper.cpp
)
target_link_libraries(lucrezio_semantic_mapping_library
  yaml-cpp
  ${catkin_LIBRARIES}
)

