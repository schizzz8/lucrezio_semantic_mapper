#pragma once

#include "detection.h"
#include "model.h"

#include <srrg_types/types.hpp>
#include <srrg_image_utils/depth_utils.h>
#include <srrg_image_utils/point_image_utils.h>

//this class implements an object detector in a simulation environment
class ObjectDetector {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ObjectDetector();

    //for each model the 3d bounding box is transformed in the rgbd camera frame
    void setupDetections();

    //performs the actual object detection
    void compute(const srrg_core::Float3Image &points_image);

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

    //output image, each pixel stores the label of the corresponding object (for visualization only)
    srrg_core::RGBImage _label_image;

  private:

    //check if a point falls into a bounding box
    inline bool inRange(const Eigen::Vector3f &point, const Eigen::Vector3f &min, const Eigen::Vector3f &max){
      return (point.x() >= min.x() && point.x() <= max.x() &&
              point.y() >= min.y() && point.y() <= max.y() &&
              point.z() >= min.z() && point.z() <= max.z());
    }

    //functions to draw label_image
    Eigen::Vector3i type2color(std::string type);
    void computeLabelImage();

};

