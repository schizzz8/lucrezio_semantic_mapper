#include "semantic_mapper_node.h"

using namespace std;
using namespace srrg_core;

SemanticMapperNode::SemanticMapperNode(ros::NodeHandle nh_):
  _nh(nh_),
  _logical_image_sub(_nh,"/gazebo/logical_camera_image",1),
  _depth_image_sub(_nh,"/camera/depth/image_raw",1),
  _rgb_image_sub(_nh,"/camera/rgb/image_raw", 1),
  _pose_sub(_nh,"/amcl_pose",1),
  _synchronizer(FilterSyncPolicy(10),_logical_image_sub,_depth_image_sub,_rgb_image_sub,_pose_sub),
  _it(_nh){

  _got_info = false;
  _camera_info_sub = _nh.subscribe("/camera/depth/camera_info",
                                   1000,
                                   &SemanticMapperNode::cameraInfoCallback,
                                   this);

  _synchronizer.registerCallback(boost::bind(&SemanticMapperNode::filterCallback, this, _1, _2, _3, _4));

  _camera_transform.setIdentity();
  _camera_transform.translation() = Eigen::Vector3f(0.0,0.0,0.6);

  _label_image_pub = _it.advertise("/camera/rgb/label_image", 1);
  _cloud_pub = _nh.advertise<PointCloud>("visualization_cloud",1);
  _marker_pub = _nh.advertise<visualization_msgs::Marker>("visualization_marker",1);
}

void SemanticMapperNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg){
  sensor_msgs::CameraInfo camerainfo;
  camerainfo.K = camera_info_msg->K;

  ROS_INFO("Got camera info!");
  _K(0,0) = camerainfo.K.c_array()[0];
  _K(0,1) = camerainfo.K.c_array()[1];
  _K(0,2) = camerainfo.K.c_array()[2];
  _K(1,0) = camerainfo.K.c_array()[3];
  _K(1,1) = camerainfo.K.c_array()[4];
  _K(1,2) = camerainfo.K.c_array()[5];
  _K(2,0) = camerainfo.K.c_array()[6];
  _K(2,1) = camerainfo.K.c_array()[7];
  _K(2,2) = camerainfo.K.c_array()[8];

  cerr << _K << endl;
  _got_info = true;
  _camera_info_sub.shutdown();
}

void SemanticMapperNode::filterCallback(const lucrezio_simulation_environments::LogicalImage::ConstPtr &logical_image_msg,
                                        const sensor_msgs::Image::ConstPtr &depth_image_msg,
                                        const sensor_msgs::Image::ConstPtr &rgb_image_msg,
                                        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg){

  if(_got_info && !logical_image_msg->models.empty()){

    //Extract rgb and depth image from ROS messages
    cv_bridge::CvImageConstPtr rgb_cv_ptr,depth_cv_ptr;
    try{
      rgb_cv_ptr = cv_bridge::toCvShare(rgb_image_msg);
      depth_cv_ptr = cv_bridge::toCvShare(depth_image_msg);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    const cv::Mat &rgb_image_ = rgb_cv_ptr->image.clone();
    const cv::Mat &raw_depth_image_ = depth_cv_ptr->image.clone();
    int rows=rgb_image_.rows;
    int cols=rgb_image_.cols;

    //extract point cloud from depth image
    srrg_core::Float3Image directions_image;
    directions_image.create(rows,cols);
    initializePinholeDirections(directions_image,_K);
    _points_image.create(rows,cols);
    computePointsImage(_points_image,
                       directions_image,
                       raw_depth_image_,
                       0.02f,
                       8.0f);

    //set models
    ModelVector models = logicalImageToModels(logical_image_msg);
    _detector.setModels(models);

    //compute detections
    _detector.setupDetections();
    _detector.compute(_points_image);
    const DetectionVector &detections = _detector.detections();

    //get camera pose
    tf::StampedTransform robot_pose;
    tf::poseMsgToTF(pose_msg->pose.pose,robot_pose);
    Eigen::Isometry3f robot_transform = tfTransform2eigen(robot_pose);

    //set globalT to mapper
    _mapper.setGlobalT(robot_transform*_camera_transform);

    //extract objects from detections
    _mapper.extractObjects(detections,_points_image);

    //data association
    _mapper.findAssociations();

    //update
    _mapper.mergeMaps();

    //publish label image
    RGBImage label_image;
    label_image.create(rows,cols);
    label_image=cv::Vec3b(0,0,0);
    makeLabelImageFromDetections(label_image,detections);
    sensor_msgs::ImagePtr label_image_msg = cv_bridge::CvImage(std_msgs::Header(),
                                                               "bgr8",
                                                               label_image).toImageMsg();
    _label_image_pub.publish(label_image_msg);

    //publish map point cloud
    PointCloud::Ptr cloud_msg (new PointCloud);
    if(_mapper.globalMap()->size()){
      makeCloudFromMap(cloud_msg,_mapper.globalMap());
      _cloud_pub.publish (cloud_msg);
    }

    //publish object bounding boxes
    visualization_msgs::Marker marker;
    if(_mapper.globalMap()->size() && _marker_pub.getNumSubscribers()){
      makeMarkerFromMap(marker,_mapper.globalMap());
      _marker_pub.publish(marker);
    }
  }
}

void SemanticMapperNode::evaluateMap(){
  std::cerr << std::endl;
  std::string path = ros::package::getPath("lucrezio_simulation_environments");
  _evaluator.setReference(path+"/config/envs/test_apartment_2/object_locations.yaml");
  _evaluator.setCurrent(_mapper.globalMap());

  _evaluator.compute();
}

Eigen::Isometry3f SemanticMapperNode::tfTransform2eigen(const tf::Transform& p){
  Eigen::Isometry3f iso;
  iso.translation().x()=p.getOrigin().x();
  iso.translation().y()=p.getOrigin().y();
  iso.translation().z()=p.getOrigin().z();
  Eigen::Quaternionf q;
  tf::Quaternion tq = p.getRotation();
  q.x()= tq.x();
  q.y()= tq.y();
  q.z()= tq.z();
  q.w()= tq.w();
  iso.linear()=q.toRotationMatrix();
  return iso;
}

tf::Transform SemanticMapperNode::eigen2tfTransform(const Eigen::Isometry3f& T){
  Eigen::Quaternionf q(T.linear());
  Eigen::Vector3f t=T.translation();
  tf::Transform tft;
  tft.setOrigin(tf::Vector3(t.x(), t.y(), t.z()));
  tft.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  return tft;
}

ModelVector SemanticMapperNode::logicalImageToModels(const lucrezio_simulation_environments::LogicalImage::ConstPtr &logical_image_msg){
  int num_models = logical_image_msg->models.size();
  ModelVector models(num_models);
  tf::StampedTransform model_pose;
  for(int i=0; i<num_models; ++i){
    models[i].type() = logical_image_msg->models[i].type;
    models[i].min() = Eigen::Vector3f(logical_image_msg->models[i].min.x,
                                      logical_image_msg->models[i].min.y,
                                      logical_image_msg->models[i].min.z);
    models[i].max() = Eigen::Vector3f(logical_image_msg->models[i].max.x,
                                      logical_image_msg->models[i].max.y,
                                      logical_image_msg->models[i].max.z);
    tf::poseMsgToTF(logical_image_msg->models[i].pose,model_pose);
    models[i].pose() = tfTransform2eigen(model_pose);

  }

  return models;
}

void SemanticMapperNode::makeLabelImageFromDetections(RGBImage &label_image, const DetectionVector &detections){
  for(int i=0; i < detections.size(); ++i){
    cv::Vec3b color(detections[i].color().x(),detections[i].color().y(),detections[i].color().z());
    for(int j=0; j < detections[i].pixels().size(); ++j){
      int r = detections[i].pixels()[j].x();
      int c = detections[i].pixels()[j].y();

      label_image.at<cv::Vec3b>(r,c) = color;
    }
  }
}

void SemanticMapperNode::makeCloudFromMap(PointCloud::Ptr &cloud, const SemanticMap *global_map){

  cloud->header.frame_id = "/map";
  cloud->height = 1;
  int num_points=0;
  for(int i=0; i < global_map->size(); ++i){
    const Eigen::Vector3f &color = global_map->at(i)->color();
    const srrg_core::Cloud3D &object_cloud = global_map->at(i)->cloud();
    for(int j=0; j < object_cloud.size(); ++j){
      pcl::PointXYZRGB point;
      point.x = object_cloud[j].point().x();
      point.y = object_cloud[j].point().y();
      point.z = object_cloud[j].point().z();
      point.r = color.z()*255;
      point.g = color.y()*255;
      point.b = color.x()*255;
      cloud->points.push_back(point);
      num_points++;
    }
  }
  cloud->width = num_points;
  pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);

}

void SemanticMapperNode::makeMarkerFromMap(visualization_msgs::Marker & marker, const SemanticMap *global_map){
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
//  marker.id = i;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  for(int i=0; i < global_map->size(); ++i){
    const ObjectPtr& object = global_map->at(i);

    marker.scale.x = 0.015;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;

    geometry_msgs::Point min,max;
    min.x = object->min().x();min.y = object->min().y();min.z = object->min().z();
    max.x = object->max().x();max.y = object->max().y();max.z = object->max().z();

    geometry_msgs::Point a,b,c,d,e,f,g,h;
    a.x=min.x;a.y=min.y;a.z=min.z;
    b.x=max.x;b.y=min.y;b.z=min.z;
    c.x=max.x;c.y=max.y;c.z=min.z;
    d.x=min.x;d.y=max.y;d.z=min.z;
    e.x=min.x;e.y=min.y;e.z=max.z;
    f.x=max.x;f.y=min.y;f.z=max.z;
    g.x=max.x;g.y=max.y;g.z=max.z;
    h.x=min.x;h.y=max.y;h.z=max.z;

    marker.points.push_back(a);
    marker.points.push_back(b);

    marker.points.push_back(b);
    marker.points.push_back(c);

    marker.points.push_back(c);
    marker.points.push_back(d);

    marker.points.push_back(d);
    marker.points.push_back(a);

    marker.points.push_back(e);
    marker.points.push_back(f);

    marker.points.push_back(f);
    marker.points.push_back(g);

    marker.points.push_back(g);
    marker.points.push_back(h);

    marker.points.push_back(h);
    marker.points.push_back(e);

    marker.points.push_back(a);
    marker.points.push_back(e);

    marker.points.push_back(b);
    marker.points.push_back(f);

    marker.points.push_back(c);
    marker.points.push_back(g);

    marker.points.push_back(d);
    marker.points.push_back(h);


  }
  marker.color.b = 1;
  marker.color.g = 0;
  marker.color.r = 0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();


}
