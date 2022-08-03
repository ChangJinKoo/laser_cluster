#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <limits.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Point.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

namespace laser_cluster{

class LaserCluster{
  public:
    explicit LaserCluster(ros::NodeHandle nh);
    ~LaserCluster();

  private:
    void init();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    sensor_msgs::PointCloud clustering(const sensor_msgs::LaserScan::ConstPtr &scan);
    static bool CmpY(pcl::PointXYZ &pt1, pcl::PointXYZ &pt2);
    pcl::PointCloud<pcl::PointXYZ> LCR(pcl::PointCloud<pcl::PointXYZ> cloud);
    
    // Node
    ros::NodeHandle nodeHandle_;
    ros::Publisher pointsPublisher_;
    ros::Subscriber scanSubscriber_;

    // clustering parameter
    float cluster_tolerance_;
    int cluster_minsize_;
    int cluster_maxsize_;
};

} /* namespace laser_cluster */
