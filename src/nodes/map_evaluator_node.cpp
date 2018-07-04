#include <iostream>
#include <yaml-cpp/yaml.h>
#include <semantic_mapper/semantic_map.h>
#include "semantic_mapper/object.cpp"

#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv){
  PointCloud::Ptr cloud (new PointCloud);

  std::string filename (argv[1]);
  std::cerr << "Reading semantic map from: " << filename << std::endl;
  YAML::Node node = YAML::LoadFile(filename);
  assert(node.IsMap());
  for(YAML::const_iterator it=node.begin(); it!=node.end(); ++it){

    const Object &obj=it->second.as<Object>();
    std::cerr << obj.model() << std::endl;
    *cloud += *obj.cloud();
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<Point> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  while (!viewer->wasStopped ()){
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;
}
