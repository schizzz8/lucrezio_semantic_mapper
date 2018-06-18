#pragma once

#include "detection.h"
#include "model.h"

//this class implements an object detector in a simulation environment
class ObjectDetector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ObjectDetector();

    //for each model the 3d bounding box is transformed in the rgbd camera frame
    void setupDetections();

    void compute(const PointCloud::ConstPtr &points);

    //setters and getters
    inline void setModels(const ModelVector &models_){_models = models_;}
    const DetectionVector &detections() const {return _detections;}

  protected:

    //camera_link to optical_frame transform
    Eigen::Isometry3f _fixed_transform;

    //vector of models detected by the logical camera
    ModelVector _models;

    //vector of detections
    DetectionVector _detections;
};

