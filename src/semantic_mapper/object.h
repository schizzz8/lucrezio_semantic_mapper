#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <srrg_types/cloud_3d.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class Object;
typedef std::shared_ptr<Object> ObjectPtr;
typedef std::vector<ObjectPtr> ObjectPtrVector;
typedef std::map<ObjectPtr,int> ObjectPtrIdMap;

//this class is a container for a 3d object that composes the semantic map
class Object {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//    //constructor
//    Object(int id_=-1,
//           const std::string &type_="",
//           const Eigen::Isometry3f &pose_=Eigen::Isometry3f::Identity(),
//           const Eigen::Vector3f &min_=Eigen::Vector3f::Zero(),
//           const Eigen::Vector3f &max_=Eigen::Vector3f::Zero(),
//           const Eigen::Vector3f &color_=Eigen::Vector3f::Zero(),
//           const srrg_core::Cloud3D &cloud_=srrg_core::Cloud3D()):
//      _id(id_),
//      _type(type_),
//      _pose(pose_),
//      _min(min_),
//      _max(max_),
//      _color(color_),
//      _cloud(cloud_){}

    //constructor
    Object(const std::string &model_="",
           const Eigen::Vector3f &position_=Eigen::Vector3f::Identity(),
           const Eigen::Vector3f &min_=Eigen::Vector3f::Zero(),
           const Eigen::Vector3f &max_=Eigen::Vector3f::Zero(),
           const Eigen::Vector3f &color_=Eigen::Vector3f::Zero(),
           const srrg_core::Cloud3D &cloud_=srrg_core::Cloud3D());

//    //setters and getters
//    inline const int id() const {return _id;}
//    inline int& id() {return _id;}
//    inline const std::string& type() const {return _type;}
//    inline std::string& type() {return _type;}
//    inline const Eigen::Isometry3f& pose() const {return _pose;}
//    inline Eigen::Isometry3f& pose() {return _pose;}
//    inline const Eigen::Vector3f& min() const {return _min;}
//    inline Eigen::Vector3f& min() {return _min;}
//    inline const Eigen::Vector3f& max() const {return _max;}
//    inline Eigen::Vector3f& max() {return _max;}
//    inline const Eigen::Vector3f &color() const {return _color;}
//    inline Eigen::Vector3f &color() {return _color;}
//    inline const srrg_core::Cloud3D &cloud() const {return _cloud;}
//    inline srrg_core::Cloud3D &cloud() {return _cloud;}

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
    inline const srrg_core::Cloud3D &cloud() const {return _cloud;}
    inline srrg_core::Cloud3D &cloud() {return _cloud;}


//    //overloaded operators
//    bool operator < (const Object &o);
//    bool operator == (const Object &o);

    //merge two objects
    void merge(const ObjectPtr &o);

  private:

//    //unique identifier (in the map) of the object
//    int _id;

//    //semantic class of the object
//    std::string _type;

    std::string _model;

//    //3D pose of the object
//    Eigen::Isometry3f _pose;

    Eigen::Vector3f _position;

    //lower vertex of the object bounding box
    Eigen::Vector3f _min;

    //upper vertex of the object bounding box
    Eigen::Vector3f _max;

    //object color (for visualization only)
    Eigen::Vector3f _color;

    //object point cloud
    srrg_core::Cloud3D _cloud;
};
