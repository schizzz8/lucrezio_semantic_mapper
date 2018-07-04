#pragma once

#include <string>
#include <map>
#include <Eigen/Geometry>
#include <semantic_mapper/semantic_map.h>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>

typedef std::map<std::string,Object> ObjectStringMap;

class MapEvaluator{
  public:
    void setReference(const std::string &filename);
    void setCurrent(const SemanticMap *current);
    void compute();
    void storeMap(const SemanticMap *current);
  protected:
    ObjectStringMap _reference;
    ObjectStringMap _current;
};
