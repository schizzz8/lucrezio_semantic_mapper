#include <iostream>
#include <fstream>

#include <object_detector/object_detector.h>
#include <semantic_mapper/semantic_mapper.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

typedef pcl::visualization::PCLVisualizer Visualizer;

typedef Eigen::Matrix<float, 7, 1> Vector7f;
Vector7f t2vFull(const Eigen::Isometry3f& iso){
  Vector7f v;
  v.head<3>() = iso.translation();
  Eigen::Quaternionf q(iso.linear());
  v(3) = q.x();
  v(4) = q.y();
  v(5) = q.z();
  v(6) = q.w();
  return v;
}


bool spin=true;

void deserializeTransform(const char * filename, Eigen::Isometry3f &transform);
void deserializeModels(const char * filename, ModelVector & models);

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

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

  //camera offset
  Eigen::Isometry3f camera_offset = Eigen::Isometry3f::Identity();
  camera_offset.linear() = Eigen::Quaternionf(0.5,-0.5,0.5,-0.5).toRotationMatrix();

  PointCloud::Ptr transformed_cloud (new PointCloud ());

  std::string line;
  std::ifstream data(argv[1]);

  Visualizer::Ptr viewer (new Visualizer ("Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem(0.25);
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
  viewer->initCameraParameters ();

  bool first=true;

  if(data.is_open()){
    while(!viewer->wasStopped()){

      if(spin && std::getline(data,line)){

        // Clear the viewer
        viewer->removeAllShapes();
        viewer->removeAllPointClouds();

        std::istringstream iss(line);

        double timestamp;
        std::string cloud_filename,transform_filename,models_filename;
        iss>>timestamp>>cloud_filename>>transform_filename>>models_filename;

        //read camera transform
        deserializeTransform(transform_filename.c_str(),camera_transform);
        std::cerr << "Camera transform: " << t2vFull(camera_transform).transpose() << std::endl;
        viewer->addCoordinateSystem(0.5,camera_transform,"camera_transform");
        mapper.setGlobalT(camera_transform);
        detector.setCameraTransform(camera_transform);

        //read cloud
        pcl::io::loadPCDFile<Point> (cloud_filename, *cloud);
        std::cerr << "Loading cloud: " << cloud_filename << std::endl;
        pcl::transformPointCloud (*cloud, *transformed_cloud, camera_transform*camera_offset);
        pcl::visualization::PointCloudColorHandlerRGBField<Point> rgb(transformed_cloud);
        viewer->addPointCloud<Point> (transformed_cloud, rgb, cloud_filename);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_filename);
        detector.setInputCloud(cloud);

        //read models
        deserializeModels(models_filename.c_str(),models);
        detector.setModels(models);
        detector.setupDetections();
        for(const Model& m : detector.models())
          viewer->addCube(m.min().x(),m.max().x(),m.min().y(),m.max().y(),m.min().z(),m.max().z(),0.0,0.0,1.0,m.type());

        //compute detections
        //      detector.compute();
        //      const DetectionVector &detections = detector.detections();

        //extract objects from detections
        //      mapper.extractObjects(detections,cloud);

        //data association
        //      mapper.findAssociations();

        //update
        //      mapper.mergeMaps();

        if(first){
          spin=!spin;
          first=false;
        }
      }

      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
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

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void){
  Visualizer::Ptr viewer = *static_cast<Visualizer::Ptr*>(viewer_void);
  if (event.getKeySym() == "p" && event.keyDown()){
    spin = !spin;
    if(spin)
      std::cerr << "PLAY" << std::endl;
    else
      std::cerr << "PAUSE" << std::endl;
  }
  //  if (event.getKeySym() == "h" && event.keyDown())
  //    std::cout << "'h' was pressed" << std::endl;
}
