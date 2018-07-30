#include <iostream>
#include <fstream>

#include <object_detector/object_detector.h>
#include <semantic_mapper/semantic_mapper.h>
#include <semantic_explorer/semantic_explorer.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>

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

void showCubes(double side,
               const PointCloud::Ptr& cloud,
               Visualizer::Ptr& viewer);

int main(int argc, char** argv){

  ObjectDetector detector;
  SemanticMapper mapper;
  SemanticExplorer explorer;

  Eigen::Isometry3f camera_transform = Eigen::Isometry3f::Identity();
  ModelVector models;
  PointCloud::Ptr cloud (new PointCloud());
  PointCloud::Ptr detection_cloud (new PointCloud());
  Point pt,origin_pt,end_pt;

  //K
  Eigen::Matrix3f K;
  K << 554.25,    0.0, 320.5,
      0.0, 554.25, 240.5,
      0.0,    0.0,   1.0;
  Eigen::Matrix3f inverse_camera_matrix = K.inverse();

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

        // read line
        std::istringstream iss(line);
        double timestamp;
        std::string cloud_filename,transform_filename,models_filename;
        iss>>timestamp>>cloud_filename>>transform_filename>>models_filename;

        //get camera transform
        deserializeTransform(transform_filename.c_str(),camera_transform);
        std::cerr << "Camera transform: " << t2vFull(camera_transform).transpose() << std::endl;
        viewer->addCoordinateSystem(0.5,camera_transform,"camera_transform");
        detector.setCameraTransform(camera_transform);
        mapper.setGlobalT(camera_transform);
        explorer.setCameraPose(camera_transform);

        //get cloud
        pcl::io::loadPCDFile<Point> (cloud_filename, *cloud);
        std::cerr << "Loading cloud: " << cloud_filename << std::endl;
        pcl::transformPointCloud (*cloud, *transformed_cloud, camera_transform*camera_offset);
        detector.setInputCloud(transformed_cloud);

        //get models
        deserializeModels(models_filename.c_str(),models);
        detector.setModels(models);
        detector.setupDetections();

        //compute detections
        detector.compute();

        const DetectionVector &detections = detector.detections();

        //update semantic map
        mapper.extractObjects(detections,cloud);
        mapper.findAssociations();
        mapper.mergeMaps();

        const SemanticMap* map = mapper.globalMap();

        //compute NBV
        explorer.setObjects(map);
        if(explorer.findNearestObject()){
          Eigen::Vector3f nbv = explorer.computeNBV();

          std::cerr << "NBV: " << nbv.transpose() << std::endl;

        }

        // Visualization

        //        for(const Model& m : detector.models())
        //          viewer->addCube(m.min().x(),m.max().x(),m.min().y(),m.max().y(),m.min().z(),m.max().z(),0.0,0.0,1.0,m.type());

        //        for(const Detection& d : detections){
        //          const std::vector<Eigen::Vector2i>& pixels = d.pixels();
        //          for(const Eigen::Vector2i& p : pixels){
        //            pt=transformed_cloud->at(p.y(),p.x());
        //            pt.r=d.color().x();
        //            pt.g=d.color().y();
        //            pt.b=d.color().z();
        //            detection_cloud->push_back(pt);
        //          }
        //        }
        //        pcl::visualization::PointCloudColorHandlerRGBField<Point> rgb(transformed_cloud);
        //        viewer->addPointCloud<Point> (transformed_cloud, rgb, cloud_filename);
        //        pcl::visualization::PointCloudColorHandlerRGBField<Point> rgb(detection_cloud);
        //        viewer->addPointCloud<Point> (detection_cloud, rgb, cloud_filename);
        //        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_filename);

        for(int i=0;i<map->size();++i){
          const ObjectPtr& obj = map->at(i);
          viewer->addCoordinateSystem (0.25,obj->position().x(),obj->position().y(),obj->position().z());
          //          viewer->addCube(obj->min().x(),obj->max().x(),
          //                          obj->min().y(),obj->max().y(),
          //                          obj->min().z(),obj->max().z(),
          //                          0.0,1.0,0.0,obj->model());
          //          showCubes(obj->s(),obj->cloud(),viewer);
          PointCloud::Ptr obj_cloud = obj->cloud();
          pcl::visualization::PointCloudColorHandlerRGBField<Point> obj_rgb(obj_cloud);
          viewer->addPointCloud<Point> (obj_cloud, obj_rgb, obj->model());
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, obj->model());
        }

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

void showCubes(double s,
               const PointCloud::Ptr& cloud,
               Visualizer::Ptr& viewer){

  // process occ cloud
  vtkSmartPointer<vtkAppendPolyData> occ_append_filter = vtkSmartPointer<vtkAppendPolyData>::New ();
  for (size_t i = 0; i < cloud->points.size (); i++) {

    if(cloud->points[i].b == 1){

      double x = cloud->points[i].x;
      double y = cloud->points[i].y;
      double z = cloud->points[i].z;

      vtkSmartPointer<vtkCubeSource> occ_cube_source = vtkSmartPointer<vtkCubeSource>::New ();

      occ_cube_source->SetBounds (x - s, x + s, y - s, y + s, z - s, z + s);
      occ_cube_source->Update ();

#if VTK_MAJOR_VERSION < 6
      occ_append_filter->AddInput (occ_cube_source->GetOutput ());
#else
      occ_append_filter->AddInputData (occ_cube_source->GetOutput ());
#endif
    }
  }

  // Remove duplicate points
  vtkSmartPointer<vtkCleanPolyData> occ_clean_filter = vtkSmartPointer<vtkCleanPolyData>::New ();
  occ_clean_filter->SetInputConnection (occ_append_filter->GetOutputPort ());
  occ_clean_filter->Update ();

  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> occ_multi_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  occ_multi_mapper->SetInputConnection (occ_clean_filter->GetOutputPort ());
  vtkSmartPointer<vtkActor> occ_multi_actor = vtkSmartPointer<vtkActor>::New ();
  occ_multi_actor->SetMapper (occ_multi_mapper);
  occ_multi_actor->GetProperty ()->SetColor (0.0, 0.0, 1.0);
  occ_multi_actor->GetProperty ()->SetAmbient (1.0);
  occ_multi_actor->GetProperty ()->SetLineWidth (1);
  occ_multi_actor->GetProperty ()->EdgeVisibilityOn ();
  occ_multi_actor->GetProperty ()->SetOpacity (1.0);
  occ_multi_actor->GetProperty ()->SetRepresentationToWireframe ();

  // Add the actor to the scene
  viewer->getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor (occ_multi_actor);

  // process unn cloud
  vtkSmartPointer<vtkAppendPolyData> unn_append_filter = vtkSmartPointer<vtkAppendPolyData>::New ();
  for (size_t i = 0; i < cloud->points.size (); i++) {

    if(cloud->points[i].g == 1){

      double x = cloud->points[i].x;
      double y = cloud->points[i].y;
      double z = cloud->points[i].z;

      vtkSmartPointer<vtkCubeSource> unn_cube_source = vtkSmartPointer<vtkCubeSource>::New ();

      unn_cube_source->SetBounds (x - s, x + s, y - s, y + s, z - s, z + s);
      unn_cube_source->Update ();

#if VTK_MAJOR_VERSION < 6
      unn_append_filter->AddInput (unn_cube_source->GetOutput ());
#else
      unn_append_filter->AddInputData (unn_cube_source->GetOutput ());
#endif
    }
  }

  // Remove duplicate points
  vtkSmartPointer<vtkCleanPolyData> unn_clean_filter = vtkSmartPointer<vtkCleanPolyData>::New ();
  unn_clean_filter->SetInputConnection (unn_append_filter->GetOutputPort ());
  unn_clean_filter->Update ();

  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> unn_multi_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  unn_multi_mapper->SetInputConnection (unn_clean_filter->GetOutputPort ());
  vtkSmartPointer<vtkActor> unn_multi_actor = vtkSmartPointer<vtkActor>::New ();
  unn_multi_actor->SetMapper (unn_multi_mapper);
  unn_multi_actor->GetProperty ()->SetColor (0.0, 1.0, 0.0);
  unn_multi_actor->GetProperty ()->SetAmbient (1.0);
  unn_multi_actor->GetProperty ()->SetLineWidth (1);
  unn_multi_actor->GetProperty ()->EdgeVisibilityOn ();
  unn_multi_actor->GetProperty ()->SetOpacity (1.0);
  unn_multi_actor->GetProperty ()->SetRepresentationToWireframe ();

  // Add the actor to the scene
  viewer->getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor (unn_multi_actor);

  // Render and interact
  viewer->getRenderWindow ()->Render ();
}
