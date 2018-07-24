#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <lucrezio_simulation_environments/LogicalImage.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <object_detector/object_detector.h>
#include <semantic_mapper/semantic_mapper.h>
#include <map_evaluator/map_evaluator.h>

#include <lucrezio_semantic_mapper/SemanticMap.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/Marker.h>
#include <pcl/io/pcd_io.h>

typedef cv::Mat_<cv::Vec3b> RGBImage;
typedef std::vector<lucrezio_simulation_environments::Model> Models;

class DataDumperNode{

public:
  DataDumperNode(ros::NodeHandle nh_,const std::string& filename):
    _nh(nh_),
    _logical_image_sub(_nh,"/gazebo/logical_camera_image",1),
    _depth_points_sub(_nh,"/camera/depth/points",1),
    _synchronizer(FilterSyncPolicy(1000),_logical_image_sub,_depth_points_sub){

    _synchronizer.registerCallback(boost::bind(&DataDumperNode::filterCallback, this, _1, _2));
    _out.open(filename);
    _seq=0;

    ROS_INFO("Starting data dumper node...");
  }

  void filterCallback(const lucrezio_simulation_environments::LogicalImage::ConstPtr &logical_image_msg,
                      const PointCloud::ConstPtr &depth_points_msg){

    //check that the there's at list one object in the robot field-of-view
    if(logical_image_msg->models.empty())
      return;

    //check that delay between messages is below a threshold
    ros::Time image_stamp = logical_image_msg->header.stamp;
    ros::Time depth_stamp;
    pcl_conversions::fromPCL(depth_points_msg->header.stamp,depth_stamp);
    ros::Duration stamp_diff = image_stamp - depth_stamp;
    if(std::abs(stamp_diff.toSec()) > 0.03)
      return;

    _last_timestamp = image_stamp;

    // save point cloud
    char cloud_filename[80];
    sprintf(cloud_filename,"cloud_%lu.png",_seq);
    pcl::io::savePCDFileASCII (cloud_filename, *depth_points_msg);

    // save camera pose
    Eigen::Isometry3f camera_transform = poseMsg2eigen(logical_image_msg->pose);
    char transform_filename[80];
    sprintf(transform_filename,"rgbd_pose_%lu.txt",_seq);
    serializeTransform(transform_filename,camera_transform);

    //save models
    const Models &models=logical_image_msg->models;
    char models_filename[80];
    sprintf(models_filename,"models_%lu.txt",_seq);
    serializeModels(models_filename,models);

    //write to output file
    _out << _last_timestamp << " ";
    _out << cloud_filename << " ";
    _out << transform_filename << " ";
    _out << models_filename << std::endl;

    _seq++;
    std::cerr << ".";

  }

  void serializeTransform(char* filename, const Eigen::Isometry3f &transform){
    std::ofstream data;
    data.open(filename);

    data << transform.translation().x() << " "
         << transform.translation().y() << " "
         << transform.translation().z() << " ";

    const Eigen::Matrix3f rotation = transform.linear().matrix();
    data << rotation(0,0) << " "
         << rotation(0,1) << " "
         << rotation(0,2) << " "
         << rotation(1,0) << " "
         << rotation(1,1) << " "
         << rotation(1,2) << " "
         << rotation(2,0) << " "
         << rotation(2,1) << " "
         << rotation(2,2) << std::endl;

    data.close();

  }

  void serializeModels(char* filename, const Models &models){
    std::ofstream data;
    data.open(filename);

    int num_models=models.size();
    data << num_models << std::endl;

    for(int i=0; i<num_models; ++i){
      const lucrezio_simulation_environments::Model &model = models[i];
      data << model.type << " ";
      tf::StampedTransform model_pose;
      tf::poseMsgToTF(model.pose,model_pose);
      const Eigen::Isometry3f model_transform=tfTransform2eigen(model_pose);
      data << model_transform.translation().x() << " "
           << model_transform.translation().y() << " "
           << model_transform.translation().z() << " ";

      const Eigen::Matrix3f model_rotation = model_transform.linear().matrix();
      data << model_rotation(0,0) << " "
           << model_rotation(0,1) << " "
           << model_rotation(0,2) << " "
           << model_rotation(1,0) << " "
           << model_rotation(1,1) << " "
           << model_rotation(1,2) << " "
           << model_rotation(2,0) << " "
           << model_rotation(2,1) << " "
           << model_rotation(2,2) << " ";

      data << model.min.x << " "
           << model.min.y << " "
           << model.min.z << " "
           << model.max.x << " "
           << model.max.y << " "
           << model.max.z << std::endl;

    }
    data.close();
  }

protected:

  //ros stuff
  ros::NodeHandle _nh;
  ros::Time _last_timestamp;

  //synchronized subscriber to rgbd frame and logical_image
  message_filters::Subscriber<lucrezio_simulation_environments::LogicalImage> _logical_image_sub;
  message_filters::Subscriber<PointCloud> _depth_points_sub;
  typedef message_filters::sync_policies::ApproximateTime<lucrezio_simulation_environments::LogicalImage,
  PointCloud> FilterSyncPolicy;
  message_filters::Synchronizer<FilterSyncPolicy> _synchronizer;

  std::ofstream _out;
  size_t _seq;

private:

  Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p){
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

  Eigen::Isometry3f poseMsg2eigen(const geometry_msgs::Pose& p){
    Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
    iso.translation().x()=p.position.x;
    iso.translation().y()=p.position.y;
    iso.translation().z()=p.position.z;
    Eigen::Quaternionf q;
    q.x()=p.orientation.x;
    q.y()=p.orientation.y;
    q.z()=p.orientation.z;
    q.w()=p.orientation.w;
    iso.linear()=q.toRotationMatrix();
    return iso;
  }
};

int main(int argc, char **argv){

  ros::init(argc, argv, "data_dumper_node");
  ros::NodeHandle nh;

  DataDumperNode dumper(nh,argv[1]);

  ros::spin();

  std::cerr << std::endl;

  return 0;
}
