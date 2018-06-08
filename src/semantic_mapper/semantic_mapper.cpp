#include "semantic_mapper.h"

using namespace srrg_core;

SemanticMapper::SemanticMapper(){

  _local_map = new SemanticMap();
  _global_map = new SemanticMap();

  _associations.clear();

  _raw_depth_scale = 0.001;
  _min_distance = 0.02;
  _max_distance = 8.0;

  _local_set = false;
  _global_set = false;

  _globalT.setIdentity();

  _fixed_transform.setIdentity();
  _fixed_transform.linear() = Eigen::Quaternionf(0.5,-0.5,0.5,-0.5).toRotationMatrix();
}

SemanticMapper::~SemanticMapper(){
  delete _local_map;
  delete _global_map;
}

ObjectPtr SemanticMapper::objectFromDetection(const Detection &detection){

  std::string type = detection.type().substr(0,detection.type().find_first_of("_"));
  std::cerr << type << ": " << std::endl;
  Eigen::Vector3f color = detection.color().cast<float>()/255.0f;
  std::cerr << "IBB: [(" << detection.topLeft().transpose() << "," << detection.bottomRight().transpose() << ")]" << std::endl;

  const std::vector<Eigen::Vector2i> &pixels = detection.pixels();
  int num_pixels = pixels.size();
  Cloud3D cloud;
  cloud.resize(num_pixels);
  int k=0;

  Eigen::Vector3f min(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
  Eigen::Vector3f max(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());

  for(int i=0; i<num_pixels; ++i){
    const cv::Vec3f& cv_point = _points_image.at<const cv::Vec3f>(pixels[i].x(), pixels[i].y());
    const cv::Vec3f& cv_normal = _normals_image.at<const cv::Vec3f>(pixels[i].x(), pixels[i].y());

    if(cv::norm(cv_point) < 1e-3 || cv::norm(cv_normal) < 0.5)
      continue;

    Eigen::Vector3f point(cv_point[0], cv_point[1],cv_point[2]);
    Eigen::Vector3f normal(cv_normal[0], cv_normal[1],cv_normal[2]);

    point = _globalT*_fixed_transform*point;

    cloud[k] = RichPoint3D(point,normal,1.0f);
    k++;

    if(point.x() < min.x())
      min.x() = point.x();
    if(point.x() > max.x())
      max.x() = point.x();
    if(point.y() < min.y())
      min.y() = point.y();
    if(point.y() > max.y())
      max.y() = point.y();
    if(point.z() < min.z())
      min.z() = point.z();
    if(point.z() > max.z())
      max.z() = point.z();
  }

  //check if object is empty
  if(!k)
    return nullptr;

  cloud.resize(k);

  //    cloud.voxelize(0.05f);

  std::cerr << "WBB: [(" << min.transpose() << "," << max.transpose() << ")]" << std::endl;

  return ObjectPtr(new Object(-1,type,Eigen::Isometry3f::Identity(),min,max,color,cloud));

}

void SemanticMapper::extractObjects(const DetectionVector &detections,
                                    const cv::Mat &depth_image){

  //the first frame populates the global map, the others populate the local map
  bool populate_global = false;
  if(!_global_set){
    populate_global = true;
    _global_set = true;
  } else {
    _local_map->clear();
    _local_set = true;
  }

  int rows=depth_image.rows;
  int cols=depth_image.cols;

  //compute points image
  srrg_core::Float3Image directions_image;
  directions_image.create(rows,cols);
  initializePinholeDirections(directions_image,_K);
  _points_image.create(rows,cols);
  computePointsImage(_points_image,
                     directions_image,
                     depth_image,
                     0.02f,
                     8.0f);

  //compute point cloud normals
  computeSimpleNormals(_normals_image,
                       _points_image,
                       3,
                       3,
                       8.0f);

  std::cerr << std::endl << "[Objects Extraction] " << std::endl;

  for(int i=0; i < detections.size(); ++i){

    const Detection& detection = detections[i];

    if(detection.pixels().size() < 10)
      continue;

    ObjectPtr obj_ptr = objectFromDetection(detection);

    if(!obj_ptr)
      continue;

    if(populate_global){
      obj_ptr->id() = _global_map->size();
      _global_map->addObject(obj_ptr);
    } else {
      _local_map->addObject(obj_ptr);
    }
  }
}

void SemanticMapper::findAssociations(){
  if(!_global_set || !_local_set)
    return;

  const int local_size = _local_map->size();
  const int global_size = _global_map->size();

  std::cerr << std::endl << "[Data Association] " << std::endl;
  std::cerr << "{Local Map size: " << local_size << "} ";
  std::cerr << "- {Global Map size: " << global_size << "}" << std::endl;

  _associations.clear();

  for(int i=0; i < global_size; ++i){
    const ObjectPtr &global = (*_global_map)[i];
    const std::string &global_type = global->type();

    std::cerr << "\t>> Global: " << global_type << "(" << global->pose().translation().transpose() << ")";

    ObjectPtr local_best = nullptr;
    float best_error = std::numeric_limits<float>::max();

    for(int j=0; j < local_size; ++j){
      const ObjectPtr &local = (*_local_map)[j];
      const std::string &local_type = local->type();

      if(local_type != global_type)
        continue;

      Eigen::Vector3f e_c = local->pose().translation() - global->pose().translation();

      float error = e_c.transpose()*e_c;

      if(error<best_error){
        best_error = error;
        local_best = local;
      }
    }

    if(!local_best){
      std::cerr << " - Local: none" << std::endl;
      continue;
    }

    std::cerr << " - Local: " << local_best->type() << "(" << local_best->pose().translation().transpose() << ")" << std::endl;
    _associations[local_best] = i;
  }
}

void SemanticMapper::mergeMaps(){
  if(!_global_set || !_local_set)
    return;

  int added = 0, merged = 0;
  std::cerr << std::endl << "[Merging] " << std::endl;

  for(int i=0; i<_local_map->size(); ++i){
    const ObjectPtr &local = (*_local_map)[i];
    ObjectPtrIdMap::iterator it = _associations.find(local);
    int association_id = -1;
    if(it != _associations.end()){
      association_id = it->second;
      ObjectPtr &global_associated = (*_global_map)[association_id];

      if(local->type() != global_associated->type())
        continue;

      global_associated->merge(local);
      merged++;
    } else {
      local->id() = _global_map->size();
      _global_map->addObject(local);
      added++;
    }
  }

  std::cerr << "merged: " << merged << std::endl;
  std::cerr << "added: " << added << std::endl;
}
