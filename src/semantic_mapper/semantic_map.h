#pragma once

#include "object.h"

//this class is a container for the semantic map
class SemanticMap : public ObjectPtrVector {
  public:

    //this function inserts an object into the map
    inline void addObject(ObjectPtr object_){push_back(object_);}

    //this function updates an object already present in the map
    inline void updateObject(ObjectPtr object_, int id){at(id)->merge(object_);}

};
