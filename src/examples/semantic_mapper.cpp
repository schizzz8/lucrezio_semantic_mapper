#include "semantic_mapper.h"

namespace lucrezio_semantic_mapping{

  using namespace std;
  using namespace lucrezio_semantic_perception;

  SemanticMapper::SemanticMapper(){
    _globalT = Eigen::Isometry3f::Identity();

    _min_distance = 0.02;
    _max_distance = 5.0;

    _local_set = false;
    _global_set = false;
  }

  Vector3fVector SemanticMapper::unproject(const std::vector<Eigen::Vector2i> &pixels){
    Vector3fVector points;
    for(int idx=0; idx < pixels.size(); ++idx){
      const Eigen::Vector2i& pixel = pixels[idx];
      int r = pixel.x();
      int c = pixel.y();
      const unsigned short& depth = _depth_image.at<const unsigned short>(r,c);
      float d = depth * _raw_depth_scale;

      if(d <= _min_distance)
        continue;

      if(d >= _max_distance)
        continue;

      Eigen::Vector3f camera_point = _invK * Eigen::Vector3f(c*d,r*d,d);
      Eigen::Vector3f map_point = _globalT*camera_point;
      //        std::cerr << map_point.transpose() << " ";
      points.push_back(map_point);

    }
    return points;
  }

  void SemanticMapper::getLowerUpper3d(const Vector3fVector &points, Eigen::Vector3f &lower, Eigen::Vector3f &upper){
    lower.x() = std::numeric_limits<float>::max();
    lower.y() = std::numeric_limits<float>::max();
    lower.z() = std::numeric_limits<float>::max();
    upper.x() = -std::numeric_limits<float>::max();
    upper.y() = -std::numeric_limits<float>::max();
    upper.z() = -std::numeric_limits<float>::max();

    for(int i=0; i < points.size(); ++i){

      if(points[i].x() < lower.x())
        lower.x() = points[i].x();
      if(points[i].x() > upper.x())
        upper.x() = points[i].x();
      if(points[i].y() < lower.y())
        lower.y() = points[i].y();
      if(points[i].y() > upper.y())
        upper.y() = points[i].y();
      if(points[i].z() < lower.z())
        lower.z() = points[i].z();
      if(points[i].z() > upper.z())
        upper.z() = points[i].z();
    }
  }

  void SemanticMapper::extractObjects(){

    bool populate_global = false;
    if(!_global_set){
      populate_global = true;
      _global_set = true;
    } else {
      _local_map.clear();
      _local_set = true;
    }

    for(int i=0; i < _detections.size(); ++i){

      const Detection& detection = _detections[i];

      cerr << detection.type() << ": [(";
      cerr << detection.topLeft().transpose() << ") - (" << detection.bottomRight().transpose() << ")]" << endl;

      if((detection.bottomRight()-detection.topLeft()).norm() < 1e-3)
        continue;

      std::cerr << "Unprojecting" << std::endl;
      Vector3fVector points = unproject(detection.pixels());

      Eigen::Vector3f lower,upper;
      getLowerUpper3d(points,lower,upper);

      Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
      pose.translation() = (upper+lower)/2.0f;

      std::string object_type = detection.type().substr(0,detection.type().find_first_of("_"));

      std::cerr << "BB: [(" << lower.transpose() << "," << upper.transpose() << ")]" << std::endl;

      if(populate_global)
        _global_map.addObject(Object(i,
                                     object_type,
                                     pose,
                                     lower,
                                     upper));
      else
        _local_map.addObject(Object(i,
                                    object_type,
                                    pose,
                                    lower,
                                    upper));
    }
  }

  void SemanticMapper::findAssociations(){

    if(!_global_set || !_local_set)
      return;

    const int local_size = _local_map.size();
    const int global_size = _global_map.size();

    std::cerr << "[Data Association] ";
    std::cerr << "{Local Map size: " << local_size << "} ";
    std::cerr << "- {Global Map size: " << global_size << "}" << std::endl;

    _associations.clear();

    for(int i=0; i < global_size; ++i){
      const Object &global = _global_map[i];
      const string &global_type = global.type();

      std::cerr << "\t>> Global: " << global_type;

      Object local_best;
      float best_error = std::numeric_limits<float>::max();

      for(int j=0; j < local_size; ++j){
        const Object &local = _local_map[j];
        const string &local_type = local.type();

        if(local_type != global_type)
          continue;

        Eigen::Vector3f e_c = local.pose().translation() - global.pose().translation();

        float error = e_c.transpose()*e_c;

        if(error<best_error){
          best_error = error;
          local_best = local;
        }
      }
      if(local_best.type() == "")
        continue;

      std::cerr << " - Local ID: " << local_best.type() << std::endl;

      _associations.push_back(Association(global,local_best));
    }
  }

  int SemanticMapper::associationID(const Object &local){
    for(int i=0; i < _associations.size(); ++i)
      if(_associations[i].second.id() == local.id())
        return _associations[i].first.id();
    return -1;
  }

  void SemanticMapper::mergeMaps(){
    if(!_global_set || !_local_set)
      return;

    int added = 0, merged = 0;
    for(int i=0; i < _local_map.size(); ++i){
      const Object &local = _local_map[i];
      int association_id = associationID(local);
      if(association_id == -1){
        _global_map.addObject(local);
        added++;
        continue;
      } else {
        Object &global_associated = _global_map[association_id];

        if(local.type() != global_associated.type())
          continue;

        global_associated.merge(local);
      }
    }
  }
}
