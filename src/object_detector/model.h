#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

class Model;
typedef std::vector<Model> ModelVector;

class Model{
  public:
    Model(std::string type_ = "",
          Eigen::Isometry3f pose_ = Eigen::Isometry3f::Identity(),
          Eigen::Vector3f min_ = Eigen::Vector3f::Zero(),
          Eigen::Vector3f max_ = Eigen::Vector3f::Zero()):
      _type(type_),
      _pose(pose_),
      _min(min_),
      _max(max_){}

    const std::string &type() const {return _type;}
    std::string &type() {return _type;}

    const Eigen::Isometry3f &pose() const {return _pose;}
    Eigen::Isometry3f &pose() {return _pose;}

    inline const Eigen::Vector3f &min() const {return _min;}
    inline Eigen::Vector3f &min() {return _min;}

    inline const Eigen::Vector3f &max() const {return _max;}
    inline Eigen::Vector3f &max() {return _max;}

  private:
    std::string _type;
    Eigen::Isometry3f _pose;
    Eigen::Vector3f _min;
    Eigen::Vector3f _max;
};

