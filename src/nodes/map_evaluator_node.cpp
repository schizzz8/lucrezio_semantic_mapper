#include <iostream>
#include <yaml-cpp/yaml.h>
#include <semantic_mapper/semantic_map.h>
#include "semantic_mapper/object.cpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>

int main(int argc, char** argv){
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  // load map
  std::string filename (argv[1]);
  YAML::Node node = YAML::LoadFile(filename);
  assert(node.IsMap());
  for(YAML::const_iterator it=node.begin(); it!=node.end(); ++it){
    const Object &obj=it->second.as<Object>();

    viewer->addCoordinateSystem (0.25,obj.position().x(),obj.position().y(),obj.position().z());

    viewer->addCube(obj.min().x(),obj.max().x(),obj.min().y(),obj.max().y(),obj.min().z(),obj.max().z(),0.0,0.0,1.0,obj.model());

    PointCloud::Ptr cloud = obj.cloud();
    pcl::visualization::PointCloudColorHandlerRGBField<Point> rgb(cloud);
    viewer->addPointCloud<Point> (cloud, rgb, obj.model());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, obj.model());
  }

  //load ground truth
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileOBJ(argv[2],mesh);
  viewer->addPolygonMesh(mesh,"meshes",0);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,"meshes");

  // init viewer
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (0.5);
  viewer->initCameraParameters ();
  while (!viewer->wasStopped ()){
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;
}
