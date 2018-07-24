#include "object_detector.h"

//using namespace srrg_core;

ObjectDetector::ObjectDetector(){
  _camera_offset.setIdentity();
  _camera_offset.linear() = Eigen::Quaternionf(0.5,-0.5,0.5,-0.5).toRotationMatrix();
//  _camera_offset.translation() = Eigen::Vector3f(0.0,0.0,0.6);
  _camera_offset_inv = _camera_offset.inverse();
}

void ObjectDetector::setupDetections(){
  if(_models.empty())
    return;

  //initialize detection vector
  _detections.clear();
  int num_models = _models.size();
  _detections.resize(num_models);

  std::vector<Eigen::Vector3f> points(2);
  for(int i=0; i<_models.size(); ++i){

    //compute model bounding box in camera frame
    points[0] = _camera_transform*_models[i].pose()*_models[i].min();
    points[1] = _camera_transform*_models[i].pose()*_models[i].max();

    float x_min=100000,x_max=-100000,y_min=100000,y_max=-100000,z_min=100000,z_max=-100000;
    for(int i=0; i < 2; ++i){
      if(points[i].x()<x_min)
        x_min = points[i].x();
      if(points[i].x()>x_max)
        x_max = points[i].x();
      if(points[i].y()<y_min)
        y_min = points[i].y();
      if(points[i].y()>y_max)
        y_max = points[i].y();
      if(points[i].z()<z_min)
        z_min = points[i].z();
      if(points[i].z()>z_max)
        z_max = points[i].z();
    }
    Eigen::Vector3f min(x_min,y_min,z_min);
    Eigen::Vector3f max(x_max,y_max,z_max);
    _models[i].min() = min;
    _models[i].max() = max;

    //set detection type
    std::string type = _models[i].type();
    _detections[i].setType(type);
  }

}

void ObjectDetector::compute(){

  if(_cloud->points.empty())
    return;

//  Point pt;
  size_t h = _cloud->height;
  size_t w = _cloud->width;
  for(size_t r=0; r<h; ++r)
    for(size_t c=0; c<w; ++c){
      const Point &p = _cloud->at(c,r);
//      pt = pcl::transformPoint(p,_camera_transform*_camera_offset);

      for(size_t i=0; i<_models.size(); ++i){
        if(_models[i].inRange(p)){

          if(r < _detections[i].topLeft().x())
            _detections[i].topLeft().x() = r;
          if(r > _detections[i].bottomRight().x())
            _detections[i].bottomRight().x() = r;

          if(c < _detections[i].topLeft().y())
            _detections[i].topLeft().y() = c;
          if(c > _detections[i].bottomRight().y())
            _detections[i].bottomRight().y() = c;

          _detections[i].pixels().push_back(Eigen::Vector2i(r,c));

          break;
        }
      }
    }
}
