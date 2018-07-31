#pragma once

#include <semantic_mapper/semantic_map.h>

typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
typedef std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> > Isometry3fVector;
typedef std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f>,
Eigen::aligned_allocator<std::pair<Eigen::Vector3f,Eigen::Vector3f> > > Vector3fPairVector;


class SemanticExplorer{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  SemanticExplorer();

  inline void setCameraPose(const Eigen::Isometry3f &camera_pose_){_camera_pose=camera_pose_;}

  void setObjects(const SemanticMap* semantic_map_);

  bool findNearestObject();

  Vector3fVector computePoses();
  Eigen::Vector3f computeNBV();

  void setProcessed();

  inline const Object* nearestObject() const {return _nearest_object;}

  inline const Vector3fPairVector& rays() const {return _rays;}

protected:
  Eigen::Isometry3f _camera_pose;
  ObjectSet _objects;
  ObjectSet _processed;
  const Object* _nearest_object;
  Vector3fPairVector _rays;

private:
  Eigen::Isometry3f transform3d(const Eigen::Vector3f& v){
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.translation() = Eigen::Vector3f(v.x(),v.y(),0.6);
    T.linear() = Eigen::AngleAxisf(v.z(),Eigen::Vector3f::UnitZ()).matrix();
  }
};
