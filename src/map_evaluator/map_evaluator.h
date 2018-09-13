#pragma once

#include <string>
#include <map>
#include <semantic_mapper/object.h>

class MapEvaluator{
public:
  void setReference(const std::string &filename);
  void setCurrent(const ObjectPtrVector *current);
  void compute();
  void storeMap(const ObjectPtrVector *current);
protected:
  GtObjectStringMap _reference;
  ObjectStringMap _current;
};
