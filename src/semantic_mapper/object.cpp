#include "object.h"

namespace YAML {
  template <typename Scalar, int Rows, int Cols>
  struct convert<Eigen::Matrix<Scalar,Rows,Cols> > {
    static Node encode(const Eigen::Matrix<Scalar,Rows,Cols> &mat){
      static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic,"Static matrices only");
      Node node(NodeType::Sequence);
      for (int i = 0; i < Rows; i++)
        for (int j = 0; j < Cols; j++)
          node[i*Cols + j] = static_cast<double>(mat(i, j));
      return node;
    }

    static bool decode(const Node &node, Eigen::Matrix<Scalar,Rows,Cols> &mat){
      static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic,"Static matrices only");
      if (!node.IsSequence() || node.size() != Rows*Cols)
        return false;
      for (int i = 0; i < Rows; i++) {
        for (int j = 0; j < Cols; j++) {
          mat(i, j) = static_cast<Scalar>(node[i * Cols + j].as<double>());
        }
      }
      return true;
    }
  };

  template<>
  struct convert<GtObject> {
    static Node encode(const GtObject &obj) {
      Node node;
      const std::string& model = obj.model();
      node["model"] = model;
      node["position"] = obj.position();
      node["orientation"] = obj.orientation();
      return node;
    }

    static bool decode(const Node& node, GtObject &obj) {
      if(!node.IsMap())
        return false;
      obj.model() = node["model"].as<std::string>();
      obj.position() = node["position"].as<Eigen::Vector3f>();
      obj.orientation() = node["orientation"].as<Eigen::Vector3f>();
      return true;
    }
  };

  template<>
  struct convert<Object> {
    static Node encode(const Object &obj) {
      Node node;
      const std::string& model = obj.model();
      node["model"] = model;
      node["position"] = obj.position();
      node["min"] = obj.min();
      node["max"] = obj.max();
      const std::string cloud_filename = model+".pcd";
      node["cloud"] = cloud_filename;
      pcl::io::savePCDFileASCII(cloud_filename,*(obj.cloud()));
      return node;
    }

    static bool decode(const Node& node, Object &obj) {
      if(!node.IsMap())
        return false;
      obj.model() = node["model"].as<std::string>();
      obj.position() = node["position"].as<Eigen::Vector3f>();
      obj.min() = node["min"].as<Eigen::Vector3f>();
      obj.max() = node["max"].as<Eigen::Vector3f>();
      const std::string cloud_filename = node["cloud"].as<std::string>();
      pcl::io::loadPCDFile<Point> (cloud_filename, *(obj.cloud()));
      return true;
    }
  };
}

using namespace std;

Object::Object(){
  _model = "";
  _position.setZero();
  _min.setZero();
  _max.setZero();
  _color.setZero();
  _cloud = PointCloud::Ptr (new PointCloud());
  _resolution = 0.05;
  _octree = Octree::Ptr (new Octree(_resolution));
  _unn_voxel_cloud = PointCloud::Ptr (new PointCloud());
  _fre_voxel_cloud = PointCloud::Ptr (new PointCloud());
  _occ_voxel_cloud = PointCloud::Ptr (new PointCloud());
  _s=0;
}

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
  _cloud(cloud_){
  _resolution = 0.05;
  _octree = Octree::Ptr (new Octree(_resolution));
  _unn_voxel_cloud = PointCloud::Ptr (new PointCloud());
  _fre_voxel_cloud = PointCloud::Ptr (new PointCloud());
  _occ_voxel_cloud = PointCloud::Ptr (new PointCloud());
  _s=0;

  _octree->setInputCloud(_cloud);
  _octree->defineBoundingBox();
  _octree->addPointsFromInputCloud();

  int depth = static_cast<int> (_octree->getTreeDepth());
  if (depth == 0)
    depth = 1;
  _s = std::sqrt (_octree->getVoxelSquaredSideLen (depth)) / 2.0;
}

bool Object::operator <(const Object &o) const{
  return (_model.compare(o.model()) < 0);
}

bool Object::operator ==(const Object &o) const{
  return (_model.compare(o.model()) == 0);
}

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

  //add new points
  *_cloud += *o->cloud();

  //voxelize
  PointCloud::Ptr cloud_filtered (new PointCloud());
  _voxelizer.setInputCloud(_cloud);
  _voxelizer.setLeafSize(0.05f,0.05f,0.05f);
  _voxelizer.filter(*cloud_filtered);
}

void Object::computeOccupancy(const Eigen::Isometry3f &T,
                              const Eigen::Vector2i & top_left,
                              const Eigen::Vector2i & bottom_right){

  if(_cloud->empty())
    return;

  std::cerr << "Computing occupancy for " << _model << "..." << std::endl;
  std::cerr << "Top left: " << top_left.transpose() << std::endl;
  std::cerr << "Bottom right: " << bottom_right.transpose() << std::endl;

  Eigen::Matrix3f K;
  K << 554.25,    0.0, 320.5,
      0.0, 554.25, 240.5,
      0.0,    0.0,   1.0;
  Eigen::Matrix3f inverse_camera_matrix = K.inverse();

  //camera offset
  Eigen::Isometry3f camera_offset = Eigen::Isometry3f::Identity();
  camera_offset.linear() = Eigen::Quaternionf(0.5,-0.5,0.5,-0.5).toRotationMatrix();

  Eigen::Vector3f origin=T*origin;
  Eigen::Vector3f end = Eigen::Vector3f::Zero();
  Octree::AlignedPointTVector voxels;
  Point pt;
  std::vector<int> indices;
//  int rows=30;
//  int cols=40;
  //  for (int r=0; r<rows; ++r)
  //    for (int c=0; c<cols; ++c){

  for (int r=top_left.x(); r<bottom_right.x(); r=r+10)
    for (int c=top_left.y(); c<bottom_right.y(); c=c+10){

      end=inverse_camera_matrix*Eigen::Vector3f(c,r,1);
      end.normalize();
      end=origin+2*end;
      end=camera_offset*end;
      end=T*end;

      voxels.clear();
      _octree->getApproxIntersectedVoxelCentersBySegment(origin, end, voxels, 0.5);

      bool hit=false;
      for(int i =0;i<voxels.size();++i){
        pt.x = voxels[i].x;
        pt.y = voxels[i].y;
        pt.z = voxels[i].z;
        pt.r = 0;
        pt.g = 0;
        pt.b = 1.0;
        indices.clear();
        bool found=_octree->voxelSearch(pt,indices);
        if(!found){
          if(hit){
            _unn_voxel_cloud->points.push_back(pt);
            continue;
          }
          _fre_voxel_cloud->points.push_back(pt);
          continue;
        }
        if(indices.size()){
          _occ_voxel_cloud->points.push_back(pt);
          if(!hit)
            hit=true;
          continue;
        }
      }
    }
  std::cerr << "occ: " << _occ_voxel_cloud->points.size() << " - ";
  std::cerr << "fre: " << _fre_voxel_cloud->points.size() << " - ";
  std::cerr << "unn: " << _unn_voxel_cloud->points.size() << std::endl;
}
