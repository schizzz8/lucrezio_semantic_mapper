add_library(map_evaluator_library SHARED
  map_evaluator.cpp map_evaluator.h
)

target_link_libraries(map_evaluator_library
  yaml-cpp
  ${catkin_LIBRARIES}
)
