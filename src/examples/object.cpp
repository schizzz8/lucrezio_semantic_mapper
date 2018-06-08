#include "object.h"

namespace lucrezio_semantic_mapping{

  using namespace std;

  Object::Object(int id_,
                 std::string type_,
                 Eigen::Isometry3f pose_,
                 Eigen::Vector3f lower_,
                 Eigen::Vector3f upper_):
    _id(id_),
    _type(type_),
    _pose(pose_),
    _lower(lower_),
    _upper(upper_){

  }

  bool Object::operator <(const Object &o){
    return (this->_id < o.id());
  }

  bool Object::operator ==(const Object &o){
    return (this->_id == o.id());
  }

  void Object::merge(const Object &o){
    if(o._lower.x() < _lower.x())
      _lower.x() = o._lower.x();
    if(o._upper.x() > _upper.x())
      _upper.x() = o._upper.x();
    if(o._lower.y() < _lower.y())
      _lower.y() = o._lower.y();
    if(o._upper.y() > _upper.y())
      _upper.y() = o._upper.y();
    if(o._lower.z() < _lower.z())
      _lower.z() = o._lower.z();
    if(o._upper.z() > _upper.z())
      _upper.z() = o._upper.z();

    _pose.translation() = (_lower+_upper)/2.0f;

  }
}

