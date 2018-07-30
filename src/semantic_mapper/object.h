#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/norms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/centroid.h>


#include <yaml-cpp/yaml.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::octree::OctreePointCloudSearch<Point> Octree;

class Object;
typedef std::shared_ptr<Object> ObjectPtr;
typedef std::vector<ObjectPtr> ObjectPtrVector;
typedef std::map<ObjectPtr,int> ObjectPtrIdMap;
typedef std::set<Object> ObjectSet;
typedef std::map<std::string,Object> ObjectStringMap;

class GtObject;
typedef std::map<std::string,GtObject> GtObjectStringMap;


//this class is a container for a 3d object that composes the semantic map
class Object {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //constructors
    Object();

    Object(const std::string &model_,
           const Eigen::Vector3f &position_,
           const Eigen::Vector3f &min_=Eigen::Vector3f::Zero(),
           const Eigen::Vector3f &max_=Eigen::Vector3f::Zero(),
           const Eigen::Vector3f &color_=Eigen::Vector3f::Zero(),
           const PointCloud::Ptr &cloud_=0);


    bool operator < (const Object &o) const;
    bool operator == (const Object &o) const;


    //setters and getters
    inline const std::string& model() const {return _model;}
    inline std::string& model() {return _model;}
    inline const Eigen::Vector3f& position() const {return _position;}
    inline Eigen::Vector3f& position() {return _position;}
    inline const Eigen::Vector3f& min() const {return _min;}
    inline Eigen::Vector3f& min() {return _min;}
    inline const Eigen::Vector3f& max() const {return _max;}
    inline Eigen::Vector3f& max() {return _max;}
    inline const Eigen::Vector3f &color() const {return _color;}
    inline Eigen::Vector3f &color() {return _color;}
    inline const PointCloud::Ptr &cloud() const {return _cloud;}
    inline PointCloud::Ptr &cloud() {return _cloud;}

    inline const PointCloud::Ptr &unnVoxelCloud() const {return _unn_voxel_cloud;}
    inline const PointCloud::Ptr &freVoxelCloud() const {return _fre_voxel_cloud;}
    inline const PointCloud::Ptr &occVoxelCloud() const {return _occ_voxel_cloud;}

    inline const float s() const {return _s;}

    //check if a point falls in the bounding box
    bool inRange(const Point &point) const;

    //merge two objects
    void merge(const ObjectPtr &o);

    //compute occupancy
    void computeOccupancy(const Eigen::Isometry3f& T,
                          const Eigen::Vector2i& top_left = Eigen::Vector2i::Zero(),
                          const Eigen::Vector2i& bottom_right = Eigen::Vector2i(480,640));

    inline const Octree::Ptr& octree() const {return _octree;}
    inline Octree::Ptr& octree() {return _octree;}

  private:

    std::string _model;

    Eigen::Vector3f _position;

    //lower vertex of the object bounding box
    Eigen::Vector3f _min;

    //upper vertex of the object bounding box
    Eigen::Vector3f _max;

    //object color (for visualization only)
    Eigen::Vector3f _color;

    //object point cloud
    PointCloud::Ptr _cloud;

    pcl::VoxelGrid<Point> _voxelizer;

    Octree::Ptr _octree;

    float _resolution;
    float _s;

    PointCloud::Ptr _occ_voxel_cloud;
    PointCloud::Ptr _fre_voxel_cloud;
    PointCloud::Ptr _unn_voxel_cloud;
};

class GtObject{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  GtObject(const std::string& model_="",
        const Eigen::Vector3f& position_ = Eigen::Vector3f::Zero(),
        const Eigen::Vector3f& orientation_ = Eigen::Vector3f::Zero()):
    _model(model_),
    _position(position_),
    _orientation(orientation_){}
  inline const std::string& model() const {return _model;}
  inline std::string& model() {return _model;}
  inline const Eigen::Vector3f& position() const {return _position;}
  inline Eigen::Vector3f& position() {return _position;}
  inline const Eigen::Vector3f& orientation() const {return _orientation;}
  inline Eigen::Vector3f& orientation() {return _orientation;}
protected:
  std::string _model;
  Eigen::Vector3f _position;
  Eigen::Vector3f _orientation;
};

