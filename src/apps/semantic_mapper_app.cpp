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
bool show_input_cloud=false;
bool show_models=false;
bool show_detections=false;
bool show_map=false;
bool show_octree=false;
bool show_rays=false;

void deserializeTransform(const char * filename, Eigen::Isometry3f &transform);
void deserializeModels(const char * filename, ModelVector & models);

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

void showInputCloud(const PointCloud::Ptr& transformed_cloud,
                    Visualizer::Ptr& viewer);

void showModels(const ModelVector& models,
                Visualizer::Ptr& viewer);

void showDetections(const PointCloud::Ptr& transformed_cloud,
                    const DetectionVector &detections,
                    Visualizer::Ptr &viewer);

void showMap(const SemanticMap* map,
             Visualizer::Ptr &viewer);

void showOctree(double side,
                const PointCloud::Ptr& occ_cloud,
                const PointCloud::Ptr& fre_cloud,
                Visualizer::Ptr& viewer);

void showRays(const Vector3fPairVector& rays,
              Visualizer::Ptr& viewer);

int main(int argc, char** argv){

  ObjectDetector detector;
  SemanticMapper mapper;
  SemanticExplorer explorer;

  Eigen::Isometry3f camera_transform = Eigen::Isometry3f::Identity();
  ModelVector models;
  PointCloud::Ptr cloud (new PointCloud());
  PointCloud::Ptr transformed_cloud (new PointCloud ());

  //camera offset
  Eigen::Isometry3f camera_offset = Eigen::Isometry3f::Identity();
  camera_offset.linear() = Eigen::Quaternionf(0.5,-0.5,0.5,-0.5).toRotationMatrix();

  //viewer
  Visualizer::Ptr viewer (new Visualizer ("Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem(0.25);
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
  viewer->initCameraParameters ();

  //read data
  bool first=true;
  std::string line;
  std::ifstream data(argv[1]);
  if(data.is_open()){
    while(!viewer->wasStopped()){

      // Clear the viewer
      viewer->removeAllShapes();
      viewer->removeAllPointClouds();

      if(spin && std::getline(data,line)){

        //parse line
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

        //update semantic map
        mapper.extractObjects(detector.detections(),cloud);
        mapper.findAssociations();
        mapper.mergeMaps();

        //compute NBV
        explorer.setObjects(mapper.globalMap());
        if(explorer.findNearestObject()){
          std::cerr << "Nearest: " << explorer.nearestObject()->model() << std::endl;
//          Eigen::Vector3f nbv = explorer.computeNBV();
//          std::cerr << "NBV: " << nbv.transpose() << std::endl;
        }
        if(first){
          spin=!spin;
          first=false;
        }
      }

      // Visualization
      if(show_input_cloud)
        showInputCloud(transformed_cloud,viewer);
      if(show_models)
        showModels(detector.models(),viewer);
      if(show_detections)
        showDetections(transformed_cloud,detector.detections(),viewer);
      if(show_map)
        showMap(mapper.globalMap(),viewer);
      if(show_octree)
        showOctree(0.05,explorer.nearestObject()->occVoxelCloud(),explorer.nearestObject()->freVoxelCloud(),viewer);
      if(show_rays)
        showRays(explorer.rays(),viewer);

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
    spin ^= 1;
    if(spin)
      std::cerr << "PLAY" << std::endl;
    else
      std::cerr << "PAUSE" << std::endl;
  }
  if (event.getKeySym() == "i" && event.keyDown()){
    show_input_cloud ^= 1;
    if(show_input_cloud)
      std::cerr << "SHOWING INPUT CLOUD" << std::endl;
    else
      std::cerr << "INPUT CLOUD HIDDEN" << std::endl;
  }
  if (event.getKeySym() == "m" && event.keyDown()){
    show_models ^= 1;
    if(show_models)
      std::cerr << "SHOWING MODELS" << std::endl;
    else
      std::cerr << "MODELS HIDDEN" << std::endl;
  }
  if (event.getKeySym() == "d" && event.keyDown()){
    show_detections ^= 1;
    if(show_detections)
      std::cerr << "SHOWING DETECTIONS" << std::endl;
    else
      std::cerr << "DETECTIONS HIDDEN" << std::endl;
  }
  if (event.getKeySym() == "s" && event.keyDown()){
    show_map ^= 1;
    if(show_map)
      std::cerr << "SHOWING SEMANTIC MAP" << std::endl;
    else
      std::cerr << "SEMANTIC MAP HIDDEN" << std::endl;
  }
  if (event.getKeySym() == "o" && event.keyDown()){
    show_octree ^= 1;
    if(show_octree)
      std::cerr << "SHOWING OCTREE" << std::endl;
    else
      std::cerr << "OCTREE HIDDEN" << std::endl;
  }
  if (event.getKeySym() == "r" && event.keyDown()){
    show_rays ^= 1;
    if(show_rays)
      std::cerr << "SHOWING RAYS" << std::endl;
    else
      std::cerr << "RAYS HIDDEN" << std::endl;
  }
  //  if (event.getKeySym() == "h" && event.keyDown())
  //    std::cout << "'h' was pressed" << std::endl;
}

void showInputCloud(const PointCloud::Ptr &transformed_cloud,
                    Visualizer::Ptr &viewer){
  pcl::visualization::PointCloudColorHandlerRGBField<Point> rgb(transformed_cloud);
  viewer->addPointCloud<Point> (transformed_cloud, rgb, "input_cloud");
}

void showModels(const ModelVector& models,
                Visualizer::Ptr& viewer){
  for(const Model& m : models)
    viewer->addCube(m.min().x(),m.max().x(),m.min().y(),m.max().y(),m.min().z(),m.max().z(),0.0,0.0,1.0,m.type());
}

void showDetections(const PointCloud::Ptr& transformed_cloud,
                    const DetectionVector &detections,
                    Visualizer::Ptr &viewer){
  PointCloud::Ptr detection_cloud (new PointCloud());
  for(const Detection& d : detections){
    Point pt;
    const std::vector<Eigen::Vector2i>& pixels = d.pixels();
    for(const Eigen::Vector2i& p : pixels){
      pt=transformed_cloud->at(p.y(),p.x());
      pt.r=d.color().x();
      pt.g=d.color().y();
      pt.b=d.color().z();
      detection_cloud->push_back(pt);
    }
  }
  pcl::visualization::PointCloudColorHandlerRGBField<Point> rgb(detection_cloud);
  viewer->addPointCloud<Point> (detection_cloud, rgb, "detection_cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "detection_cloud");
}

void showMap(const SemanticMap *map,
             Visualizer::Ptr &viewer){
  for(int i=0;i<map->size();++i){
    const ObjectPtr& obj = map->at(i);
    viewer->addCoordinateSystem (0.25,obj->position().x(),obj->position().y(),obj->position().z());
    viewer->addCube(obj->min().x(),obj->max().x(),
                    obj->min().y(),obj->max().y(),
                    obj->min().z(),obj->max().z(),
                    0.0,1.0,0.0,obj->model());
    PointCloud::Ptr obj_cloud = obj->cloud();
    pcl::visualization::PointCloudColorHandlerRGBField<Point> obj_rgb(obj_cloud);
    viewer->addPointCloud<Point> (obj_cloud, obj_rgb, obj->model());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, obj->model());
  }
}

void showOctree(double s,
                const PointCloud::Ptr& occ_cloud,
                const PointCloud::Ptr& fre_cloud,
                Visualizer::Ptr& viewer){

  if(occ_cloud->empty() && occ_cloud->empty())
    return;

  if(occ_cloud->points.size()){
    // process occ cloud
    vtkSmartPointer<vtkAppendPolyData> occ_append_filter = vtkSmartPointer<vtkAppendPolyData>::New ();
    for (size_t i = 0; i < occ_cloud->points.size (); i++) {
      double x = occ_cloud->points[i].x;
      double y = occ_cloud->points[i].y;
      double z = occ_cloud->points[i].z;

      vtkSmartPointer<vtkCubeSource> occ_cube_source = vtkSmartPointer<vtkCubeSource>::New ();

      occ_cube_source->SetBounds (x - s, x + s, y - s, y + s, z - s, z + s);
      occ_cube_source->Update ();

#if VTK_MAJOR_VERSION < 6
      occ_append_filter->AddInput (occ_cube_source->GetOutput ());
#else
      occ_append_filter->AddInputData (occ_cube_source->GetOutput ());
#endif
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
    occ_multi_actor->GetProperty ()->SetColor (1.0, 0.0, 0.0);
    occ_multi_actor->GetProperty ()->SetAmbient (1.0);
    occ_multi_actor->GetProperty ()->SetLineWidth (1);
    occ_multi_actor->GetProperty ()->EdgeVisibilityOn ();
    occ_multi_actor->GetProperty ()->SetOpacity (1.0);
    occ_multi_actor->GetProperty ()->SetRepresentationToWireframe ();

    // Add the actor to the scene
    viewer->getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor (occ_multi_actor);
  }

  if(fre_cloud->points.size()){
    // process fre cloud
    vtkSmartPointer<vtkAppendPolyData> fre_append_filter = vtkSmartPointer<vtkAppendPolyData>::New ();
    for (size_t i = 0; i < fre_cloud->points.size (); i++) {
      double x = fre_cloud->points[i].x;
      double y = fre_cloud->points[i].y;
      double z = fre_cloud->points[i].z;

      vtkSmartPointer<vtkCubeSource> fre_cube_source = vtkSmartPointer<vtkCubeSource>::New ();

      fre_cube_source->SetBounds (x - s, x + s, y - s, y + s, z - s, z + s);
      fre_cube_source->Update ();

#if VTK_MAJOR_VERSION < 6
      fre_append_filter->AddInput (fre_cube_source->GetOutput ());
#else
      fre_append_filter->AddInputData (fre_cube_source->GetOutput ());
#endif
    }

    // Remove duplicate points
    vtkSmartPointer<vtkCleanPolyData> fre_clean_filter = vtkSmartPointer<vtkCleanPolyData>::New ();
    fre_clean_filter->SetInputConnection (fre_append_filter->GetOutputPort ());
    fre_clean_filter->Update ();

    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> fre_multi_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    fre_multi_mapper->SetInputConnection (fre_clean_filter->GetOutputPort ());
    vtkSmartPointer<vtkActor> fre_multi_actor = vtkSmartPointer<vtkActor>::New ();
    fre_multi_actor->SetMapper (fre_multi_mapper);
    fre_multi_actor->GetProperty ()->SetColor (0.0, 1.0, 0.0);
    fre_multi_actor->GetProperty ()->SetAmbient (1.0);
    fre_multi_actor->GetProperty ()->SetLineWidth (1);
    fre_multi_actor->GetProperty ()->EdgeVisibilityOn ();
    fre_multi_actor->GetProperty ()->SetOpacity (1.0);
    fre_multi_actor->GetProperty ()->SetRepresentationToWireframe ();

    // Add the actor to the scene
    viewer->getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor (fre_multi_actor);
  }

  // Render and interact
  viewer->getRenderWindow ()->Render ();
}

void showRays(const Vector3fPairVector &rays,
              Visualizer::Ptr &viewer){
  for(int i=0; i<rays.size(); ++i){
    char buffer[30];
    sprintf(buffer,"line_%d",i);
    Eigen::Vector3f origin=rays[i].first;
    Eigen::Vector3f end=rays[i].second;
    Point o;
    o.x=origin.x();
    o.y=origin.y();
    o.z=origin.z();
    Point e;
    e.x=end.x();
    e.y=end.y();
    e.z=end.z();
    viewer->addLine(o,e,buffer);
  }
}
