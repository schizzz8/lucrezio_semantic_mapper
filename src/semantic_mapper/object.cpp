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
//      obj.position() = node["position"].as<Eigen::Vector3f>();
      obj.orientation() = node["orientation"].as<Eigen::Vector3f>();
      Eigen::Vector3f min = node["min"].as<Eigen::Vector3f>();
      Eigen::Vector3f max = node["max"].as<Eigen::Vector3f>();
      obj.min() = min;
      obj.max() = max;
      obj.position() = (min+max)/2.0f;
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

Object::Object():_octree(new octomap::OcTree(0.05)){
  _model = "";
  _position.setZero();
  _min.setZero();
  _max.setZero();
  _color.setZero();
  _cloud = PointCloud::Ptr (new PointCloud());
  _fre_voxel_cloud = PointCloud::Ptr (new PointCloud());
  _occ_voxel_cloud = PointCloud::Ptr (new PointCloud());
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
  _cloud(cloud_),
  _octree(new octomap::OcTree(0.05)),
  _fre_voxel_cloud(new PointCloud()),
  _occ_voxel_cloud(new PointCloud()){}

Object::Object(const string &model_,
               const Eigen::Vector3f &position_,
               const Eigen::Vector3f &min_,
               const Eigen::Vector3f &max_,
               const Eigen::Vector3f &color_,
               const string &cloud_filename,
               const string &octree_filename,
               const string &fre_voxel_cloud_filename,
               const string &occ_voxel_cloud_filename):
  _model(model_),
  _position(position_),
  _min(min_),
  _max(max_),
  _color(color_),
  _cloud(new PointCloud()),
  _fre_voxel_cloud(new PointCloud()),
  _occ_voxel_cloud(new PointCloud()){

  pcl::io::loadPCDFile<Point> (cloud_filename, *_cloud);

  _octree = new octomap::OcTree(octree_filename);

  if(fre_voxel_cloud_filename != "...")
    pcl::io::loadPCDFile<Point> (fre_voxel_cloud_filename, *_fre_voxel_cloud);

  if(occ_voxel_cloud_filename != "...")
    pcl::io::loadPCDFile<Point> (occ_voxel_cloud_filename, *_occ_voxel_cloud);

}

Object::Object(const Object &obj):
  _model(obj.model()),
  _position(obj.position()),
  _min(obj.min()),
  _max(obj.max()),
  _color(obj.color()),
  _cloud(obj.cloud()),
  _octree(obj.octree()),
  _fre_voxel_cloud(obj.freVoxelCloud()),
  _occ_voxel_cloud(obj.occVoxelCloud()){}

Object::~Object(){
  delete _octree;
}

bool Object::operator <(const Object &o) const{
  return (_model.compare(o.model()) < 0);
}

bool Object::operator ==(const Object &o) const{
  return (_model.compare(o.model()) == 0);
}

bool Object::inRange(const Point &point) const{
  return (point.x >= _min.x() && point.x <= _max.x() &&
          point.y >= _min.y() && point.y <= _max.y() &&
          point.z >= _min.z() && point.z <= _max.z());
}

bool Object::inRange(const float &x, const float &y, const float &z) const{
  return (x >= _min.x() && x <= _max.x() &&
          y >= _min.y() && y <= _max.y() &&
          z >= _min.z() && z <= _max.z());
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
//  _voxelizer.setLeafSize(0.05f,0.05f,0.05f);
  _voxelizer.setLeafSize(0.02f,0.02f,0.02f);
  _voxelizer.filter(*cloud_filtered);

  //update cloud
  _cloud->clear();
  *_cloud = *cloud_filtered;
}

void Object::updateOccupancy(const Eigen::Isometry3f &T, const PointCloud::Ptr & cloud){

  if(cloud->empty())
    return;

  octomap::Pointcloud scan;
  for(const Point& pt : cloud->points)
    scan.push_back(pt.x,pt.y,pt.z);

  octomap::point3d origin(T.translation().x(),T.translation().y(),T.translation().z());
  _octree->insertPointCloud(scan,origin);

  octomap::point3d p;
  Point pt;
  _occ_voxel_cloud->clear();
  _fre_voxel_cloud->clear();
  for(octomap::OcTree::tree_iterator it = _octree->begin_tree(_octree->getTreeDepth()),end=_octree->end_tree(); it!= end; ++it) {
    if (it.isLeaf()) {
      p = it.getCoordinate();

      if(!inRange(p.x(),p.y(),p.z()))
        continue;

      if (_octree->isNodeOccupied(*it)){ // occupied voxels
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        _occ_voxel_cloud->points.push_back(pt);
      }
      else { // free voxels
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        _fre_voxel_cloud->points.push_back(pt);
      }
    }
  }
  _fre_voxel_cloud->width = _fre_voxel_cloud->size();
  _fre_voxel_cloud->height = 1;

  _occ_voxel_cloud->width = _occ_voxel_cloud->size();
  _occ_voxel_cloud->height = 1;
}
