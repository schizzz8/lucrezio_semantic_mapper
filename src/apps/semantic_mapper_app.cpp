#include <iostream>
#include <fstream>

#include <object_detector/object_detector.h>
#include <semantic_mapper/semantic_mapper.h>

void deserializeTransform(const char * filename, Eigen::Isometry3f &transform);
void deserializeModels(const char * filename, ModelVector & models);

int main(int argc, char** argv){

  ObjectDetector detector;
  SemanticMapper mapper;

  Eigen::Isometry3f camera_transform = Eigen::Isometry3f::Identity();
  ModelVector models;
  PointCloud::Ptr cloud (new PointCloud());

  //K
  Eigen::Matrix3f K;
  K << 554.25,    0.0, 320.5,
      0.0, 554.25, 240.5,
      0.0,    0.0,   1.0;

  std::string line;
  std::ifstream data(argv[1]);

  bool first = true;
  if(data.is_open()){
    while(std::getline(data,line) && first){

      std::istringstream iss(line);
      double timestamp;
      std::string cloud_filename,transform_filename,models_filename;
      iss>>timestamp>>cloud_filename>>transform_filename>>models_filename;

      //read cloud
      pcl::io::loadPCDFile<Point> ("test_pcd.pcd", *cloud);

      //read camera transform
      deserializeTransform(transform_filename.c_str(),camera_transform);
      mapper.setGlobalT(camera_transform);

      //read models
      deserializeModels(models_filename.c_str(),models);
      detector.setModels(models);

      //compute detections
      detector.setupDetections();
      detector.compute(cloud);
      const DetectionVector &detections = detector.detections();

      //extract objects from detections
      mapper.extractObjects(detections,cloud);

      //data association
      mapper.findAssociations();

      //update
      mapper.mergeMaps();




    }
  }


  return 0;
}

void deserializeTransform(const char * filename, Eigen::Isometry3f &transform){
  std::ifstream fin(filename);
  std::string line;

  transform.setIdentity();
  if(fin.is_open()){
    if(std::getline(fin,line)){
      std::istringstream iss(line);
      double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
      iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
      transform.translation()=Eigen::Vector3f(px,py,pz);
      Eigen::Matrix3f R;
      R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
      transform.linear().matrix() = R;
    }
  }

  fin.close();
}


void deserializeModels(const char * filename, ModelVector & models){
  std::ifstream fin(filename);
  std::string line;

  models.clear();
  if(fin.is_open()){
    while(std::getline(fin,line)){
      std::istringstream iss(line);
      std::string type;
      double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
      double minx,miny,minz,maxx,maxy,maxz;
      iss >> type;

      if(type.length() < 3)
        continue;

      Eigen::Isometry3f model_pose=Eigen::Isometry3f::Identity();
      iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
      model_pose.translation()=Eigen::Vector3f(px,py,pz);
      Eigen::Matrix3f R;
      R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
      model_pose.linear().matrix() = R;
      iss >> minx>>miny>>minz>>maxx>>maxy>>maxz;
      Eigen::Vector3f min(minx,miny,minz);
      Eigen::Vector3f max(maxx,maxy,maxz);

      models.push_back(Model(type,model_pose,min,max));
    }
  }

  fin.close();
}
