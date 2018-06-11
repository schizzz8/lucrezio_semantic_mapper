#include "object.h"


using namespace std;

Object::Object(const string &model_,
               const Eigen::Vector3f &position_,
               const Eigen::Vector3f &min_,
               const Eigen::Vector3f &max_,
               const Eigen::Vector3f &color_,
               const srrg_core::Cloud3D &cloud_):
  _model(model_),
  _position(position_),
  _min(min_),
  _max(max_),
  _color(color_),
  _cloud(cloud_){}

//bool Object::operator <(const Object &o){
//  return (this->_id < o.id());
//}

//bool Object::operator ==(const Object &o){
//  return (this->_id == o.id());
//}

void Object::merge(const ObjectPtr & o){
  if(o->min().x() < _min.x())
    _min.x() = o->min().x();
  if(o->max().x() > _max.x())
    _max.x() = o->max().x();
  if(o->min().y() < _min.y())
    _min.y() = o->min().y();
  if(o->max().y() > _max.y())
    _max.y() = o->max().y();
  if(o->min().z() < _min.z())
    _min.z() = o->min().z();
  if(o->max().z() > _max.z())
    _max.z() = o->max().z();

  _position = (_min+_max)/2.0f;

  //    _cloud.add(o->cloud());
  //    _cloud.voxelize(0.05f);

  _cloud = o->cloud();

}

