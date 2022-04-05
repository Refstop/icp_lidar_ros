#include "icp_lidar_ros/icp_lidar_ros.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "icp_lidar");
    icp_lidar_ros icp;
    icp.run();
    return 0;
}