#include "object.h"


using namespace std;

Object::Object(const string &model_,
               const Eigen::Vector3f &position_,
               const Eigen::Vector3f &min_,
               const Eigen::Vector3f &max_,
               const Eigen::Vector3f &color_,
               const PointCloud::Ptr & cloud_):
  _model(model_),
  _position(position_),
  _min(min_),
  _max(max_),
  _color(color_),
  _cloud(cloud_){}

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

  *_cloud += *o->cloud();

//  pcl::GeneralizedIterativeClosestPoint<Point,Point> gicp;
//  gicp.setInputSource(o->cloud());
//  gicp.setInputTarget(_cloud);
//  PointCloud::Ptr output(new PointCloud());
//  gicp.align(*output);

//  *_cloud += *output;
}

