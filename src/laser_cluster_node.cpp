#include "laser_cluster/laser_cluster.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_cluster_node");
  ros::NodeHandle nodeHandle("~");
  laser_cluster::LaserCluster LC(nodeHandle);
  ros::spin();
  return 0;
}
