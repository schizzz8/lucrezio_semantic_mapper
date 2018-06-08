#include "object_detector.h"

using namespace srrg_core;

ObjectDetector::ObjectDetector(){
  _fixed_transform.setIdentity();
  _fixed_transform.linear() = Eigen::Quaternionf(0.5,-0.5,0.5,-0.5).toRotationMatrix();
  _fixed_transform = _fixed_transform.inverse();
}

void ObjectDetector::setupDetections(){

  //initialize detection vector
  int num_models = _models.size();
  _detections.resize(num_models);

  std::vector<Eigen::Vector3f> points(2);
  for(int i=0; i<_models.size(); ++i){

    //compute model bounding box in camera frame
    points[0] = _fixed_transform*_models[i].pose()*_models[i].min();
    points[1] = _fixed_transform*_models[i].pose()*_models[i].max();

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
    _models[i].min() = Eigen::Vector3f(x_min,y_min,z_min);
    _models[i].max() = Eigen::Vector3f(x_max,y_max,z_max);

    //set detection type and color
    std::string type = _models[i].type();
    std::string detection_type = type.substr(0,type.find_first_of("_"));
    _detections[i].type() = detection_type;
    const Eigen::Vector3i color = type2color(detection_type);
    _detections[i].color() = color;

    //init detection
    _detections[i].topLeft() = Eigen::Vector2i(10000,10000);
    _detections[i].bottomRight() = Eigen::Vector2i(-10000,-10000);

  }
}

void ObjectDetector::computeImageBoundingBoxes(){

  for(int r=0; r<_points_image.rows; ++r){
    const cv::Vec3f* point_ptr=_points_image.ptr<const cv::Vec3f>(r);
    const cv::Vec3f* normal_ptr=_normals_image.ptr<const cv::Vec3f>(r);
    for(int c=0; c<_points_image.cols; ++c, ++point_ptr, ++normal_ptr){
      const cv::Vec3f& p = *point_ptr;
      const cv::Vec3f& n = *normal_ptr;

      if(cv::norm(p) < 1e-3 || cv::norm(n) < 0.5)
        continue;

      const Eigen::Vector3f point(p[0],p[1],p[2]);

      for(int j=0; j < _models.size(); ++j){

        if(inRange(point,_models[j].min(),_models[j].max())){

          if(r < _detections[j].topLeft().x())
            _detections[j].topLeft().x() = r;
          if(r > _detections[j].bottomRight().x())
            _detections[j].bottomRight().x() = r;

          if(c < _detections[j].topLeft().y())
            _detections[j].topLeft().y() = c;
          if(c > _detections[j].bottomRight().y())
            _detections[j].bottomRight().y() = c;

          _detections[j].pixels().push_back(Eigen::Vector2i(r,c));

          break;
        }
      }
    }
  }
}

DetectionVector ObjectDetector::compute(const cv::Mat &rgb_image_,
                                        const cv::Mat &raw_depth_image_){

  int rows=rgb_image_.rows;
  int cols=rgb_image_.cols;

  _detections.clear();

  //compute points image
  srrg_core::Float3Image directions_image;
  directions_image.create(rows,cols);
  initializePinholeDirections(directions_image,_K);
  _points_image.create(rows,cols);
  computePointsImage(_points_image,
                     directions_image,
                     raw_depth_image_,
                     0.02f,
                     8.0f);

  //compute point cloud normals
  computeSimpleNormals(_normals_image,
                       _points_image,
                       3,
                       3,
                       8.0f);

  double cv_time = (double)cv::getTickCount();
  setupDetections();

  computeImageBoundingBoxes();
  printf("Computing BB took: %f\n",((double)cv::getTickCount() - cv_time)/cv::getTickFrequency());

  //Compute label image (for visualization only)
  _label_image.create(rows,cols);
  _label_image=cv::Vec3b(0,0,0);
  computeLabelImage();

  return _detections;
}

Eigen::Vector3i ObjectDetector::type2color(std::string type){
  int c;

  if(type == "sink")
    c = 1;
  if(type == "burner")
    c = 2;
  if(type == "table")
    c = 3;
  if(type == "chair")
    c = 4;
  if(type == "couch")
    c = 5;
  if(type == "bed")
    c = 6;
  if(type == "cabinet")
    c = 7;
  if(type == "bathroom")
    c = 8;
  if(type == "milk")
    c = 9;
  if(type == "salt")
    c = 10;
  if(type == "tomato")
    c = 11;
  if(type == "zwieback")
    c = 12;

  std::stringstream stream;
  stream << std::setw(6) << std::setfill('0') << std::hex << ((float)c/12.0f)*16777215;
  std::string result(stream.str());

  unsigned long r_value = std::strtoul(result.substr(0,2).c_str(), 0, 16);
  unsigned long g_value = std::strtoul(result.substr(2,2).c_str(), 0, 16);
  unsigned long b_value = std::strtoul(result.substr(4,2).c_str(), 0, 16);

  return Eigen::Vector3i(r_value,g_value,b_value);
}

void ObjectDetector::computeLabelImage(){

  for(int i=0; i < _detections.size(); ++i){
    cv::Vec3b color(_detections[i].color().x(),_detections[i].color().y(),_detections[i].color().z());
    for(int j=0; j < _detections[i].pixels().size(); ++j){
      int r = _detections[i].pixels()[j].x();
      int c = _detections[i].pixels()[j].y();

      _label_image.at<cv::Vec3b>(r,c) = color;
    }
  }
}

