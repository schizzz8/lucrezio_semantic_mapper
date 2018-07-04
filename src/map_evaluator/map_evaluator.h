#pragma once

#include <string>
#include <map>
#include <semantic_mapper/semantic_map.h>

class MapEvaluator{
public:
  void setReference(const std::string &filename);
  void setCurrent(const SemanticMap *current);
  void compute();
  void storeMap(const SemanticMap *current);
protected:
  GtObjectStringMap _reference;
  ObjectStringMap _current;
};
