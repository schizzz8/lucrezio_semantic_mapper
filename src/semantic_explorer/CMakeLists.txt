add_library(semantic_explorer_library SHARED
  semantic_explorer.cpp semantic_explorer.h
)

target_link_libraries(semantic_explorer_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
