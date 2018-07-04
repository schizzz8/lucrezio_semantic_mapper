#include "map_evaluator.h"

namespace YAML {
  template <typename Scalar, int Rows, int Cols>
  struct convert<Eigen::Matrix<Scalar,Rows,Cols> > {
    static Node encode(const Eigen::Matrix<Scalar,Rows,Cols> &mat){
      static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic,"Static matrices only");
      Node node(NodeType::Sequence);
      for (int i = 0; i < Rows; i++)
        for (int j = 0; j < Cols; j++)
          node[i*Cols + j] = static_cast<double>(mat(i, j));
      return node;
    }

    static bool decode(const Node &node, Eigen::Matrix<Scalar,Rows,Cols> &mat){
      static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic,"Static matrices only");
      if (!node.IsSequence() || node.size() != Rows*Cols)
        return false;
      for (int i = 0; i < Rows; i++) {
        for (int j = 0; j < Cols; j++) {
          mat(i, j) = static_cast<Scalar>(node[i * Cols + j].as<double>());
        }
      }
      return true;
    }
  };

  template<>
  struct convert<Object> {
    static Node encode(const Object &obj) {
      Node node;
      const std::string& model = obj.model();
      node["model"] = model;
      node["position"] = obj.position();
      node["min"] = obj.min();
      node["max"] = obj.max();
      const std::string cloud_filename = model+".pcd";
      node["cloud"] = cloud_filename;
      pcl::io::savePCDFileASCII(cloud_filename,*(obj.cloud()));
      return node;
    }

    static bool decode(const Node& node, Object &obj) {
      if(!node.IsMap())
        return false;
      obj.model() = node["model"].as<std::string>();
      obj.position() = node["position"].as<Eigen::Vector3f>();
//      obj.min() = node["min"].as<Eigen::Vector3f>();
//      obj.max() = node["max"].as<Eigen::Vector3f>();
//      const std::string cloud_filename = node["cloud"].as<std::string>();
//      pcl::io::loadPCDFile<Point> (cloud_filename, *(obj.cloud()));
      return true;
    }
  };

}

void MapEvaluator::setReference(const std::string & filename){
  if(filename.empty())
    return;

  _reference.clear();

  YAML::Node map = YAML::LoadFile(filename);

  for(YAML::const_iterator it=map.begin(); it!=map.end(); ++it){
    const std::string &key=it->first.as<std::string>();
    const Object &value=it->second.as<Object>();

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
  for(ObjectStringMap::iterator it = _reference.begin(); it != _reference.end(); ++it){
    const std::string &reference_model = it->first;
    const Object &reference_object = it->second;

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

  YAML::Node node;
  for(size_t i=0; i<current->size(); ++i){
    const std::string &key = current->at(i)->model();
    const Object &obj = *(current->at(i));
    node[key]=obj;
  }

  assert(node.IsMap());


  std::ofstream fout("cazzo.yaml");
  fout << node;
}

