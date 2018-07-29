#include "semantic_explorer.h"

void SemanticExplorer::setup(){
  _robot_pose.setIdentity();
  _objects.clear();
  _processed.clear();
  _nearest_object = 0;
}

void SemanticExplorer::setObjects(const SemanticMap &semantic_map_){
  for(size_t i=0; i<semantic_map_.size(); ++i){
    const Object &o = *(semantic_map_[i]);

    if(o.model() != "table_ikea_bjursta" && o.model() != "couch")
      continue;

    ObjectSet::iterator it = _processed.find(o);
    if(it!=_processed.end())
      continue;

    it = _objects.find(o);
    if(it!=_objects.end())
      continue;

    _objects.insert(o);
  }
}

bool SemanticExplorer::findNearestObject(){
  if(_nearest_object)
    throw std::runtime_error("[SemanticExplorer][findNearestObject]: you're messing up things!");

  float min_dist = std::numeric_limits<float>::max();

  for(ObjectSet::iterator it=_objects.begin(); it!=_objects.end(); ++it){
    const Object& o = *it;
    float dist=(o.position()-_robot_pose.translation()).norm();
    if(dist<min_dist){
      min_dist=dist;
      _nearest_object=&o;
    }
  }

  return _nearest_object;
}

Vector3fVector SemanticExplorer::computePoses(){
  if(!_nearest_object)
    throw std::runtime_error("[SemanticExplorer][computePoses]: no nearest object!");

  Vector3fVector poses(4);
  poses[0] = Eigen::Vector3f(_nearest_object->position().x()+1.0,_nearest_object->position().y(),M_PI);
  poses[1] = Eigen::Vector3f(_nearest_object->position().x(),_nearest_object->position().y()+1.0,-M_PI_2);
  poses[2] = Eigen::Vector3f(_nearest_object->position().x()-1.0,_nearest_object->position().y(),0);
  poses[3] = Eigen::Vector3f(_nearest_object->position().x(),_nearest_object->position().y()-1.0,M_PI_2);

  return poses;
}

void SemanticExplorer::setProcessed(){
  if(!_nearest_object)
    throw std::runtime_error("[SemanticExplorer][setProcessed]: no nearest object!");

  ObjectSet::iterator it = _objects.find(*_nearest_object);
  if(it!=_objects.end()){
    Object o = *it;
    _processed.insert(o);
    _objects.erase(it);
  } else
    throw std::runtime_error("[SemanticExplorer][setProcessed]: you're messing up things!");

  _nearest_object = 0;
}
