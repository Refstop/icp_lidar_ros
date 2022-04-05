#ifndef ICP_LIDAR
#define ICP_LIDAR

#include <iostream>
#include <icp_lidar_ros/knncpp.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#define RAD2DEG(rad) rad*(180/M_PI)
#define DEG2RAD(deg) deg*(M_PI/180)

using namespace std;
using namespace Eigen;

typedef knncpp::Matrixi Matrixi;

class icp_lidar_ros {
    public:
    MatrixXd reference_points, points_to_be_aligned;
    icp_lidar_ros();
    ~icp_lidar_ros() {}
    void Scan1Callback(const sensor_msgs::LaserScan::ConstPtr& msg); // reference_points
    void Scan2Callback(const sensor_msgs::LaserScan::ConstPtr& msg); // points_to_be_aligned
    void knn_kdtree(MatrixXd reference_points, MatrixXd points_to_be_aligned);
    double* point_based_matching(MatrixXd points_pair_a, MatrixXd points_pair_b);
    MatrixXd icp(MatrixXd reference_points, MatrixXd points, int max_iterations=100, float distance_threshold=0.3, float convergence_translation_threshold=1e-3,
        float convergence_rotation_threshold=1e-4, int point_pairs_threshold=10, bool verbose=true);
    void run();

    private:
    Matrixi indices_;
    MatrixXd distances_;
    ros::NodeHandle nh_;
    ros::Subscriber scan1_sub_, scan2_sub_;
    ros::Publisher aligned_scan_pub_;
    float angle_max_, angle_min_, angle_increment_, range_min_, range_max_;
    bool scan1, scan2;
    Vector2d Split_(string input, char delimiter);
    void push_back_(MatrixXd& m, Vector2d&& values, std::size_t row);
};

#endif