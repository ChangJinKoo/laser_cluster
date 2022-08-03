#include "laser_cluster/laser_cluster.hpp"

using namespace std;

namespace laser_cluster{

LaserCluster::LaserCluster(ros::NodeHandle nh)
  : nodeHandle_(nh){
  init();
}

LaserCluster::~LaserCluster(){
  nodeHandle_.deleteParam("cluster_params/tolerance");
  nodeHandle_.deleteParam("cluster_params/minsize");
  nodeHandle_.deleteParam("cluster_params/maxsize");
}

void LaserCluster::init(){
  // get parameters
  nodeHandle_.param("cluster_params/tolerance", cluster_tolerance_, 0.4f);
  nodeHandle_.param("cluster_params/minsize", cluster_minsize_, 3);
  nodeHandle_.param("cluster_params/maxsize", cluster_maxsize_, 100);

  // set subscriber
  scanSubscriber_ = nodeHandle_.subscribe("/scan", 5, &LaserCluster::scanCallback, this);

  // set publisher
  pointsPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud>("/preceding_truck_points", 5);
}

void LaserCluster::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  sensor_msgs::PointCloud result;

  result = clustering(scan);
  result.header = scan->header;

  pointsPublisher_.publish(result); 
}

sensor_msgs::PointCloud LaserCluster::clustering(const sensor_msgs::LaserScan::ConstPtr &scan) {
  // sensor_msgs::LaserScan -> sensor_msgs::PointCloud ================
  sensor_msgs::PointCloud msgCloud;
  laser_geometry::LaserProjection projector_;

  projector_.projectLaser(*scan, msgCloud);

  pcl::PointCloud<pcl::PointXYZ> inputCloud;
  inputCloud.width = msgCloud.points.size();
  inputCloud.height = 1;
  inputCloud.points.resize(inputCloud.width * inputCloud.height);
  for (int i = 0; i < msgCloud.points.size(); i++)
  {
    inputCloud.points[i].x = msgCloud.points[i].x;
    inputCloud.points[i].y = msgCloud.points[i].y;
    inputCloud.points[i].z = 0;
  }

  // kdtree cluster
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  kdtree->setInputCloud(inputCloud.makeShared());

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  // kdtree threshold
  ec.setClusterTolerance(cluster_tolerance_);

  // size < min_value -> noise -> not cluster
  ec.setMinClusterSize(cluster_minsize_);
  ec.setMaxClusterSize(cluster_maxsize_);

  ec.setSearchMethod(kdtree);
  ec.setInputCloud(inputCloud.makeShared());

  ec.extract(clusterIndices);

  sensor_msgs::PointCloud result;
  result.points.resize(3);
  double min_dist = DBL_MAX;

  for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ> tmpCloud;

    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      tmpCloud.push_back(inputCloud[*pit]);
    }

    pcl::PointCloud<pcl::PointXYZ> extracted_pt = LCR(tmpCloud);

    if (sqrt(pow(extracted_pt[1].x, 2) + pow(extracted_pt[1].y, 2)) < min_dist) {
      result.points[0].x = extracted_pt[0].x;
      result.points[0].y = -1 * extracted_pt[0].y;
      result.points[0].z = extracted_pt[0].z;

      result.points[1].x = extracted_pt[1].x;
      result.points[1].y = -1 * extracted_pt[1].y;
      result.points[1].z = extracted_pt[1].z;

      result.points[2].x = extracted_pt[2].x;
      result.points[2].y = -1 * extracted_pt[2].y;
      result.points[2].z = extracted_pt[2].z;

      min_dist = sqrt(pow(extracted_pt[1].x, 2) + pow(extracted_pt[1].y, 2));
    }
  }

  return result;
}

bool LaserCluster::CmpY(pcl::PointXYZ &pt1, pcl::PointXYZ &pt2) {
  return pt1.y < pt2.y;
}

pcl::PointCloud<pcl::PointXYZ> LaserCluster::LCR(pcl::PointCloud<pcl::PointXYZ> cloud) {
  pcl::PointCloud<pcl::PointXYZ> result;
  pcl::PointXYZ Pt_center;

  sort(cloud.begin(), cloud.end(), LaserCluster::CmpY);

  Pt_center.x = (cloud[0].x + cloud[cloud.size()-1].x) / 2;
  Pt_center.y = (cloud[0].y + cloud[cloud.size()-1].y) / 2;
  Pt_center.z = 0; 

  result.push_back(cloud[0]);
  result.push_back(Pt_center);
  result.push_back(cloud[cloud.size()-1]);

  return result;
}

} /* namespace laser_cluster */
