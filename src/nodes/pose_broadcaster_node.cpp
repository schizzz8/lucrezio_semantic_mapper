#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/LinkStates.h>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <lucrezio_simulation_environments/LogicalImage.h>

class PoseBroadcaster{
public:
  PoseBroadcaster(ros::NodeHandle nh_):
    _nh(nh_),
    _logical_image_sub(_nh,"/gazebo/logical_camera_image",1),
    _scan_sub(_nh,"/scan",1),
    _synchronizer(FilterSyncPolicy(1000),_logical_image_sub,_scan_sub){

//    _camera_pose_sub = _nh.subscribe("/gazebo/link_states",
//                                     1000,
//                                     &PoseBroadcaster::cameraPoseCallback,
//                                     this);

//    _scan_sub = _nh.subscribe("/scan",
//                              1000,
//                              &PoseBroadcaster::scanCallback,
//                              this);

    _synchronizer.registerCallback(boost::bind(&PoseBroadcaster::filterCallback, this, _1, _2));
  }

//  void cameraPoseCallback(const gazebo_msgs::LinkStates::ConstPtr& camera_pose_msg){
//    const std::vector<std::string> &names = camera_pose_msg->name;
//    for(size_t i=0; i<names.size(); ++i){
//      if(names[i].compare("robot::camera_link") == 0){
//        const geometry_msgs::Pose camera_pose = camera_pose_msg->pose[i];
//        _camera_transform=poseMsg2eigen(camera_pose);
//        break;
//      }
//    }
//  }

//  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_msg){

//    ros::Time stamp = laser_msg->header.stamp;

//    tf::StampedTransform camera_tf;

//    try{
//      _listener.waitForTransform("/odom",
//                                 "/camera_link",
//                                 stamp,
//                                 ros::Duration(0.5));
//      _listener.lookupTransform("/odom",
//                                "/camera_link",
//                                stamp,
//                                camera_tf);
//    } catch (tf::TransformException ex){
//      ROS_ERROR("%s",ex.what());
//    }
//    Eigen::Isometry3f camera_transform = tfTransform2eigen(camera_tf);

//    tf::Transform odom_to_map_tf = eigen2tfTransform(_camera_transform*camera_transform.inverse());
//    _br.sendTransform(tf::StampedTransform(odom_to_map_tf, stamp, "/map", "/odom"));
//  }

  void filterCallback(const lucrezio_simulation_environments::LogicalImage::ConstPtr &logical_image_msg,
                      const sensor_msgs::LaserScan::ConstPtr &laser_msg){
    ros::Time laser_stamp = laser_msg->header.stamp;
    tf::StampedTransform camera_odom_tf;
    try{
      _listener.waitForTransform("/odom",
                                 "/camera_link",
                                 laser_stamp,
                                 ros::Duration(0.5));
      _listener.lookupTransform("/odom",
                                "/camera_link",
                                laser_stamp,
                                camera_odom_tf);
    } catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    Eigen::Isometry3f camera_odom_transform = tfTransform2eigen(camera_odom_tf);

    ros::Time image_stamp = logical_image_msg->header.stamp;
    Eigen::Isometry3f camera_map_transform = poseMsg2eigen(logical_image_msg->pose);

    tf::Transform odom_map_tf = eigen2tfTransform(camera_map_transform*camera_odom_transform.inverse());
    _br.sendTransform(tf::StampedTransform(odom_map_tf, image_stamp, "/map", "/odom"));

    ROS_INFO("Laser msg stamp: %f", laser_stamp.toSec());
    ROS_INFO("Logical image stamp: %f", image_stamp.toSec());
    ROS_INFO("...");
  }

  Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p){
    Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
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

  Eigen::Isometry3f poseMsg2eigen(const geometry_msgs::Pose &p){
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

  tf::Transform eigen2tfTransform(const Eigen::Isometry3f& T){
    Eigen::Quaternionf q(T.linear());
    Eigen::Vector3f t=T.translation();
    tf::Transform tft;
    tft.setOrigin(tf::Vector3(t.x(), t.y(), t.z()));
    tft.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    return tft;
  }

protected:
  ros::NodeHandle _nh;
  tf::TransformListener _listener;

  tf::TransformBroadcaster _br;

//  ros::Subscriber _scan_sub;

//  ros::Subscriber _camera_pose_sub;
  Eigen::Isometry3f _camera_transform;

  message_filters::Subscriber<lucrezio_simulation_environments::LogicalImage> _logical_image_sub;
  message_filters::Subscriber<sensor_msgs::LaserScan> _scan_sub;
  typedef message_filters::sync_policies::ApproximateTime<lucrezio_simulation_environments::LogicalImage,
  sensor_msgs::LaserScan> FilterSyncPolicy;
  message_filters::Synchronizer<FilterSyncPolicy> _synchronizer;

};

int main(int argc, char **argv){

  ros::init(argc, argv, "pose_broadcaster_node");
  ros::NodeHandle nh;

  PoseBroadcaster dummy(nh);

  ros::spin();

  return 0;
}
