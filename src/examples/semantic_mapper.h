#pragma once

#include <iostream>

#include "semantic_map.h"

#include <lucrezio_semantic_perception/detection.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace lucrezio_semantic_mapping {

  typedef std::vector<Eigen::Vector3f> Vector3fVector;

class SemanticMapper{
public:
    SemanticMapper();

    void setDetections(const lucrezio_semantic_perception::DetectionVector &detections_){_detections = detections_;}

    void extractObjects();
    void findAssociations();
    void mergeMaps();

protected:
    float _raw_depth_scale;
    float _min_distance, _max_distance;
    Eigen::Matrix3f _K,_invK;
    cv::Mat _depth_image;
    lucrezio_semantic_perception::DetectionVector _detections;

    bool _local_set;
    bool _global_set;

    SemanticMap _local_map;
    SemanticMap _global_map;
    Eigen::Isometry3f _globalT;

    std::vector<Association> _associations;

private:
    Vector3fVector unproject(const std::vector<Eigen::Vector2i> &pixels);
    void getLowerUpper3d(const Vector3fVector &points, Eigen::Vector3f &lower, Eigen::Vector3f &upper);
    void unproject();
    int associationID(const Object &local);
};

}
