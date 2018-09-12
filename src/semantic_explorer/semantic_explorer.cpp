#include "semantic_explorer.h"

SemanticExplorer::SemanticExplorer(){
  _camera_pose.setIdentity();
  _objects.clear();
  _processed.clear();
  _nearest_object = 0;
}

void SemanticExplorer::setObjects(const SemanticMap* semantic_map_){
  for(size_t i=0; i<semantic_map_->size(); ++i){
    const Object &o = *(semantic_map_->at(i));

    //    if(o.model() != "table_ikea_bjursta" && o.model() != "couch")
    //      continue;

    ObjectSet::iterator it = _processed.find(o);
    if(it!=_processed.end())
      continue;

    it = _objects.find(o);
    if(it!=_objects.end())
      continue;

    _objects.insert(o);
  }
}

bool SemanticExplorer::findNearestObject(){
  if(_nearest_object)
    throw std::runtime_error("[SemanticExplorer][findNearestObject]: you're messing up things!");

  float min_dist = std::numeric_limits<float>::max();

  for(ObjectSet::iterator it=_objects.begin(); it!=_objects.end(); ++it){
    const Object& o = *it;
    float dist=(o.position()-_camera_pose.translation()).norm();
    if(dist<min_dist){
      min_dist=dist;
      _nearest_object=&o;
    }
  }
  return _nearest_object;
}

Vector3fVector SemanticExplorer::computePoses(){
  if(!_nearest_object)
    throw std::runtime_error("[SemanticExplorer][computePoses]: no nearest object!");

  Vector3fVector poses(4);
  poses[0] = Eigen::Vector3f(_nearest_object->position().x()+1.0,_nearest_object->position().y(),M_PI);
  poses[1] = Eigen::Vector3f(_nearest_object->position().x(),_nearest_object->position().y()+1.0,-M_PI_2);
  poses[2] = Eigen::Vector3f(_nearest_object->position().x()-1.0,_nearest_object->position().y(),0);
  poses[3] = Eigen::Vector3f(_nearest_object->position().x(),_nearest_object->position().y()-1.0,M_PI_2);

  return poses;
}

Eigen::Vector3f SemanticExplorer::computeNBV(){
  if(!_nearest_object)
    throw std::runtime_error("[SemanticExplorer][computeNBV]: no nearest object!");

  Eigen::Vector3f nbv = Eigen::Vector3f::Zero();
  int unn_max=-1;

  Vector3fPairVector rays;

  //K
  Eigen::Matrix3f K;
  K << 554.25,    0.0, 320.5,
      0.0, 554.25, 240.5,
      0.0,    0.0,   1.0;
  Eigen::Matrix3f inverse_camera_matrix = K.inverse();

  //camera offset
  Eigen::Isometry3f camera_offset = Eigen::Isometry3f::Identity();
  camera_offset.linear() = Eigen::Quaternionf(0.5,-0.5,0.5,-0.5).toRotationMatrix();

  //simulate view
  for(int i=-1; i<=1; ++i)
    for(int j=-1; j<=1; ++j){

      if(i==0 && j==0)
        continue;
      if(i!=0 && j!=0)
        continue;

      //computing view pose
      Eigen::Isometry3f T=Eigen::Isometry3f::Identity();
      Eigen::Vector3f v;
      v << _nearest_object->position().x() + i,
          _nearest_object->position().y() + j,
          std::atan2(-j,-i);
      T.translation() = Eigen::Vector3f(v.x(),v.y(),0.6);
      T.linear() = Eigen::AngleAxisf(v.z(),Eigen::Vector3f::UnitZ()).matrix();

      //set ray origin to camera pose
      octomap::point3d origin(v.x(),v.y(),0.6);
      std::cerr << v.transpose() << " - ";

      //generate rays
      Eigen::Vector3f end = Eigen::Vector3f::Zero();
      int occ=0,fre=0,unn=0;
      std::vector<octomap::point3d> ray;
      for (int r=0; r<480; r=r+40)
        for (int c=0; c<640; c=c+40){

          //compute ray endpoint
          end=inverse_camera_matrix*Eigen::Vector3f(c,r,1);
          end.normalize();
          end=5*end;
          end=camera_offset*end;
          end=T*end;
          octomap::point3d dir(end.x(),end.y(),end.z());

          //store ray
          rays.push_back(std::make_pair(T.translation(),end));

          //ray casting
          ray.clear();
          if(_nearest_object->octree()->computeRay(origin,dir,ray)){
            for(const octomap::point3d voxel : ray){

              if(!_nearest_object->inRange(voxel.x(),voxel.y(),voxel.z()))
                continue;

              octomap::OcTreeNode* n = _nearest_object->octree()->search(voxel);
              if(n){
                double value = n->getOccupancy();
                if(value>0.5)
                  occ++;
                else
                  fre++;
                break;
              } else
                unn++;
            }
          }
        }
      std::cerr << std::endl << "occ: " << occ << " - unn: " << unn << " - fre: " << fre << std::endl;

      if(unn>unn_max){
        unn_max = unn;
        nbv=v;
        _rays = rays;
      }
      rays.clear();
    }
  return nbv;
}

void SemanticExplorer::setProcessed(){
  if(!_nearest_object)
    throw std::runtime_error("[SemanticExplorer][setProcessed]: no nearest object!");

  ObjectSet::iterator it = _objects.find(*_nearest_object);
  if(it!=_objects.end()){
    Object o = *it;
    _processed.insert(o);
    _objects.erase(it);
  } else
    throw std::runtime_error("[SemanticExplorer][setProcessed]: you're messing up things!");

  _nearest_object = 0;
}
