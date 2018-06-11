#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <lucrezio_simulation_environments/LogicalImage.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <object_detector/object_detector.h>
#include <semantic_mapper/semantic_mapper.h>
#include <map_evaluator/map_evaluator.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class SemanticMapperNode{

  public:
    SemanticMapperNode(ros::NodeHandle nh_);

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);

    void filterCallback(const lucrezio_simulation_environments::LogicalImage::ConstPtr &logical_image_msg,
                        const sensor_msgs::Image::ConstPtr &depth_image_msg,
                        const sensor_msgs::Image::ConstPtr &rgb_image_msg,
                        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg);

    void evaluateMap();

  protected:
    ros::NodeHandle _nh;

    ros::Subscriber _camera_info_sub;
    bool _got_info;
    bool _first = true;

    //synchronized subscriber to rgbd frame and logical_image
    message_filters::Subscriber<lucrezio_simulation_environments::LogicalImage> _logical_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> _depth_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> _rgb_image_sub;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> _pose_sub;
    typedef message_filters::sync_policies::ApproximateTime<lucrezio_simulation_environments::LogicalImage,
    sensor_msgs::Image,
    sensor_msgs::Image,
    geometry_msgs::PoseWithCovarianceStamped> FilterSyncPolicy;
    message_filters::Synchronizer<FilterSyncPolicy> _synchronizer;

    ObjectDetector _detector;
    SemanticMapper _mapper;
    MapEvaluator _evaluator;

    //rgbd camera matrix
    Eigen::Matrix3f _K;

    //camera pose w.r.t robot base link
    Eigen::Isometry3f _camera_transform;

    //organized point cloud obtained from the depth image
    srrg_core::Float3Image _points_image;

    //publisher for the label image (visualization only)
    image_transport::ImageTransport _it;
    image_transport::Publisher _label_image_pub;
    ros::Publisher _cloud_pub;
    ros::Publisher _marker_pub;

  private:
    //extract models from logical image msg
    ModelVector logicalImageToModels(const lucrezio_simulation_environments::LogicalImage::ConstPtr &logical_image_msg);

    Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p);
    tf::Transform eigen2tfTransform(const Eigen::Isometry3f& T);

    void makeLabelImageFromDetections(srrg_core::RGBImage &label_image, const DetectionVector &detections);

    void makeCloudFromMap(PointCloud::Ptr &cloud, const SemanticMap *global_map);

    void makeMarkerFromMap(visualization_msgs::Marker &marker, const SemanticMap *global_map);
};
