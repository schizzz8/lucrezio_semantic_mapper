#pragma once

#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <srrg_types/types.hpp>
#include <srrg_image_utils/depth_utils.h>
#include <srrg_image_utils/point_image_utils.h>

#include <object_detector/object_detector.h>

#include "semantic_map.h"

class SemanticMapper{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SemanticMapper();

    virtual ~SemanticMapper();

    //set camera matrix
    inline void setK(const Eigen::Matrix3f& K_){_K = K_; _invK = _K.inverse();}

    //set robot pose
    inline void setGlobalT(const Eigen::Isometry3f &globalT_){_globalT = globalT_;}

    //specialized extractObjects method
    void extractObjects(const DetectionVector &detections,
                        const cv::Mat &depth_image_);

    //specialized findAssociations method
    void findAssociations();

    //specialized mergeMaps method
    void mergeMaps();

  protected:

    //organized point cloud obtained from the depth image
    srrg_core::Float3Image _points_image;

    //point cloud normals
    srrg_core::Float3Image _normals_image;

    //pose of the robot w.r.t. the global map
    Eigen::Isometry3f _globalT;

    Eigen::Isometry3f _fixed_transform;

    //flags
    bool _local_set;
    bool _global_set;

    //map built from the current frame
    SemanticMap *_local_map;

    //actual map that stores objects in a global reference frame and gets updated for each new observation
    SemanticMap *_global_map;

    //this map stores the output of the data-association
    ObjectPtrIdMap _associations;

  private:

    //rgbd camera parameters
    float _raw_depth_scale;
    float _min_distance, _max_distance;

    //rgbd camera matrix
    Eigen::Matrix3f _K,_invK;

    //this function builds an object from the detector output
    ObjectPtr objectFromDetection(const Detection &detection);
};
