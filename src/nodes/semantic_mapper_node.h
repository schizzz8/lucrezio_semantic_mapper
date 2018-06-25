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

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <gazebo_msgs/LinkStates.h>

#include <gazebo_msgs/GetLinkState.h>

#include <geometry_msgs/Twist.h>

typedef cv::Mat_<cv::Vec3b> RGBImage;

class SemanticMapperNode{

  public:
    SemanticMapperNode(ros::NodeHandle nh_);

    void filterCallback(const lucrezio_simulation_environments::LogicalImage::ConstPtr &logical_image_msg,
                        const PointCloud::ConstPtr &depth_points_msg);

    void evaluateMap();

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

    Eigen::Isometry3f _camera_transform;

    //computing modules
    ObjectDetector _detector;
    SemanticMapper _mapper;
    MapEvaluator _evaluator;

    //publisher for the label image (visualization only)
    image_transport::ImageTransport _it;
    image_transport::Publisher _label_image_pub;
    ros::Publisher _cloud_pub;
    ros::Publisher _marker_pub;

  private:
    //extract models from logical image msg
    ModelVector logicalImageToModels(const lucrezio_simulation_environments::LogicalImage::ConstPtr &logical_image_msg);

    Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p);
    Eigen::Isometry3f poseMsg2eigen(const geometry_msgs::Pose& p);
    tf::Transform eigen2tfTransform(const Eigen::Isometry3f& T);

    void makeLabelImageFromDetections(sensor_msgs::ImagePtr &label_image_msg, const DetectionVector &detections);

    void makeCloudFromMap(PointCloud::Ptr &cloud, const SemanticMap *global_map);

    void makeMarkerFromMap(visualization_msgs::Marker &marker, const SemanticMap *global_map);
};
