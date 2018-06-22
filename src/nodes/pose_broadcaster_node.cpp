#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/LinkStates.h>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>

class PoseBroadcaster{
public:
  PoseBroadcaster(ros::NodeHandle nh_):
    _nh(nh_){

    _camera_pose_sub = _nh.subscribe("/gazebo/link_states",
                                     1000,
                                     &PoseBroadcaster::cameraPoseCallback,
                                     this);
    _scan_sub = _nh.subscribe("/scan",
                              1000,
                              &PoseBroadcaster::scanCallback,
                              this);
  }

  void cameraPoseCallback(const gazebo_msgs::LinkStates::ConstPtr& camera_pose_msg){
    const std::vector<std::string> &names = camera_pose_msg->name;
    for(size_t i=0; i<names.size(); ++i){
      if(names[i].compare("robot::camera_link") == 0){
        const geometry_msgs::Pose camera_pose = camera_pose_msg->pose[i];
        _camera_transform=poseMsg2eigen(camera_pose);
        break;
      }
    }
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &laser_msg){

//    ROS_INFO("Now: %f",ros::Time::now().toSec());

    ros::Time stamp = laser_msg->header.stamp;
//    ROS_INFO("Laser msg timestamp: %f",stamp.toSec());

    tf::StampedTransform camera_tf;

    try{
      _listener.waitForTransform("/odom",
                                 "/camera_link",
                                 stamp,
                                 ros::Duration(0.5));
      _listener.lookupTransform("/odom",
                                "/camera_link",
                                stamp,
                                camera_tf);
    } catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
//    ROS_INFO("Camera tf timestamp: %f",camera_tf.stamp_.toSec());
    Eigen::Isometry3f camera_transform = tfTransform2eigen(camera_tf);

    tf::Transform odom_to_map_tf = eigen2tfTransform(_camera_transform*camera_transform.inverse());
    _br.sendTransform(tf::StampedTransform(odom_to_map_tf, stamp, "/map", "/odom"));
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

  ros::Subscriber _scan_sub;

  ros::Subscriber _camera_pose_sub;
  Eigen::Isometry3f _camera_transform;

};

int main(int argc, char **argv){

  ros::init(argc, argv, "pose_broadcaster_node");
  ros::NodeHandle nh;

  PoseBroadcaster dummy(nh);

  ros::spin();

  return 0;
}

//  ros::ServiceClient link_state_client;
//  link_state_client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

//  Eigen::Isometry3f map_camera_transform;

//  tf::TransformBroadcaster br;

//  ros::Rate rate(100);
//  while(ros::ok()){

//    ros::Time now = ros::Time::now();
//    try{
//      odom_listener.waitForTransform("/odom",
//                                     "/camera_link",
//                                     now,
//                                     ros::Duration(0.5));
//      odom_listener.lookupTransform("/odom",
//                                    "/camera_link",
//                                    now,
//                                    odom_camera_tf);
//    } catch (tf::TransformException ex){
//      ROS_ERROR("%s",ex.what());
//    }
//    Eigen::Isometry3f odom_camera_transform = tfTransform2eigen(odom_camera_tf);

//    gazebo_msgs::GetLinkState link_state;
//    link_state.request.link_name = "robot::camera_link";
//    if(link_state_client.call(link_state)){
//      ROS_INFO("Received camera_link state!\n");
//      map_camera_transform=poseMsg2eigen(link_state.response.link_state.pose);
//    }else
//      ROS_ERROR("Failed to call service gazebo/get_model_state");

//    tf::Transform transform = eigen2tfTransform(map_camera_transform*odom_camera_transform.inverse());

//    br.sendTransform(tf::StampedTransform(transform, now, "/map", "/odom"));


//    ROS_INFO("Broadcasting transform");

//    ros::spinOnce();
//    rate.sleep();
//  }
