#include "map_evaluator.h"
#include "semantic_mapper/object.cpp"

void MapEvaluator::setReference(const std::string & filename){
  if(filename.empty())
    return;

  _reference.clear();

  YAML::Node map = YAML::LoadFile(filename);

  for(YAML::const_iterator it=map.begin(); it!=map.end(); ++it){
    const std::string &key=it->first.as<std::string>();
    const GtObject &value=it->second.as<GtObject>();

    _reference.insert(std::make_pair(key,value));
  }
}

void MapEvaluator::setCurrent(const SemanticMap *current){
  if(!current)
    return;

  _current.clear();

  for(size_t i=0; i<current->size(); ++i){
    const std::string &key = current->at(i)->model();
    const Object &value = *(current->at(i));

    _current.insert(std::make_pair(key,value));
  }
}

void MapEvaluator::compute(){
  for(GtObjectStringMap::iterator it = _reference.begin(); it != _reference.end(); ++it){
    const std::string &reference_model = it->first;
    const GtObject &reference_object = it->second;

    ObjectStringMap::iterator jt = _current.find(reference_model);
    if(jt!=_current.end()){
      //model found
      std::cerr << reference_model << ": found!" << std::endl;
      const Object &current_object = jt->second;

      //evaluate object
      const Eigen::Vector3f &ref_pos = reference_object.position();
      const Eigen::Vector3f &cur_pos = current_object.position();
      float error = (ref_pos - cur_pos).norm();
      std::cerr << "\t>>position error: " << error << std::endl;
    }
  }
}

void MapEvaluator::storeMap(const SemanticMap *current){
  if(!current)
    return;

  YAML::Node node(YAML::NodeType::Sequence);
  for(size_t i=0; i<current->size(); ++i){
    const std::string &key = current->at(i)->model();
    const Object &obj = *(current->at(i));
    node[key]=obj;
  }

  std::ofstream fout("semantic_map.yaml");
  fout << node;
}
