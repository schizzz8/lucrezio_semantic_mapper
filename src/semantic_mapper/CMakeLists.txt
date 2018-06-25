add_library(semantic_mapper_library SHARED
  object.h object.cpp
  semantic_map.h
  semantic_mapper.h semantic_mapper.cpp
)

target_link_libraries(semantic_mapper_library
  utils_library
  object_detector_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

