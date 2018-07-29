#pragma once

#include <semantic_mapper/semantic_map.h>

typedef std::vector<Eigen::Vector3f> Vector3fVector;

class SemanticExplorer{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  void setup();

  inline void setRobotPose(const Eigen::Isometry3f &robot_pose_){_robot_pose=robot_pose_;}

  void setObjects(const SemanticMap &semantic_map_);

  bool findNearestObject();

  Vector3fVector computePoses();

  void setProcessed();

protected:
  Eigen::Isometry3f _robot_pose;
  ObjectSet _objects;
  ObjectSet _processed;
  const Object* _nearest_object;
};
