#include "icp_lidar_ros/icp_lidar_ros.h"

icp_lidar_ros::icp_lidar_ros(): scan1(false), scan2(false) {
    scan1_sub_ = nh_.subscribe("scan1", 1, &icp_lidar_ros::Scan1Callback, this);
    scan2_sub_ = nh_.subscribe("scan2", 1, &icp_lidar_ros::Scan2Callback, this);
    aligned_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
}

void icp_lidar_ros::Scan1Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    reference_points.resize(1,2);
    std::vector<float> ranges = msg->ranges;
    angle_max_ = msg->angle_max;
    angle_min_ = msg->angle_min;
    angle_increment_ = msg->angle_increment;
    range_min_ = msg->range_min;
    range_max_ = msg->range_max;
    int row = 0;
    for(int i = 0; i < ranges.size(); i++) {
        if(ranges[i] == 0) continue;
        push_back_(reference_points, Vector2d(ranges[i]*cos(-angle_min_ - i*angle_increment_), ranges[i]*sin(-angle_min_ - i*angle_increment_)), row);
        row++;
    }
    reference_points.transposeInPlace();
    scan1 = true;
}

void icp_lidar_ros::Scan2Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    points_to_be_aligned.resize(1,2);
    std::vector<float> ranges = msg->ranges;
    int row = 0;
    for(int i = 0; i < ranges.size(); i++) {
        if(ranges[i] == 0) continue;
        push_back_(points_to_be_aligned, Vector2d(ranges[i]*cos(-angle_min_ - i*angle_increment_), ranges[i]*sin(-angle_min_ - i*angle_increment_)), row);
        row++;
    }
    points_to_be_aligned.transposeInPlace();
    scan2 = true;
}

void icp_lidar_ros::knn_kdtree(const MatrixXd reference_points, const MatrixXd points_to_be_aligned) {
    knncpp::KDTreeMinkowskiX<double, knncpp::EuclideanDistance<double>> kdtree(reference_points);

    kdtree.setBucketSize(1);
    kdtree.build();

    kdtree.query(points_to_be_aligned, 1, indices_, distances_);
}

double* icp_lidar_ros::point_based_matching(const MatrixXd points_pair_a, const MatrixXd points_pair_b) {
    static double xytheta[3];
    double x, y, xp, yp, x_mean = 0, y_mean = 0, xp_mean = 0, yp_mean = 0;
    int n = points_pair_a.rows();
    if(n == 0) return NULL;
    for(int i = 0; i < n; i++) {
        // a_mean += points_pair_a.block<1,2>(i,0);
        // b_mean += points_pair_b.block<1,2>(i,0);
        x = points_pair_a(i,0);
        y = points_pair_a(i,1);
        xp = points_pair_b(i,0);
        yp = points_pair_b(i,1);
        

        x_mean += x;
        y_mean += y;
        xp_mean += xp;
        yp_mean += yp;
    }

    x_mean /= n;
    y_mean /= n;
    xp_mean /= n;
    yp_mean /= n;
    
    double s_x_xp = 0, s_y_yp = 0, s_x_yp = 0, s_y_xp = 0;
    for(int i = 0; i < n; i++) {
        x = points_pair_a(i,0);
        y = points_pair_a(i,1);
        xp = points_pair_b(i,0);
        yp = points_pair_b(i,1);

        s_x_xp += (x - x_mean)*(xp - xp_mean);
        s_y_yp += (y - y_mean)*(yp - yp_mean);
        s_x_yp += (x - x_mean)*(yp - yp_mean);
        s_y_xp += (y - y_mean)*(xp - xp_mean);
    }

    xytheta[2] = atan2(s_x_yp - s_y_xp, s_x_xp + s_y_yp);
    xytheta[0] = xp_mean - (x_mean*cos(xytheta[2]) - y_mean*sin(xytheta[2]));
    xytheta[1] = yp_mean - (x_mean*sin(xytheta[2]) + y_mean*cos(xytheta[2]));

    return xytheta;
}

MatrixXd icp_lidar_ros::icp(MatrixXd reference_points, MatrixXd points, int max_iterations, float distance_threshold, float convergence_translation_threshold,
        float convergence_rotation_threshold, int point_pairs_threshold, bool verbose) {
    for(int iter_num = 0; iter_num < max_iterations; iter_num++) {
        if(verbose) cout << "------ iteration " << iter_num << " ------" << endl;
        knn_kdtree(reference_points, points);
        // cout << "distances_:" << endl << distances_ << endl << endl;
        // cout << "indices_:" << endl << indices_ << endl << endl;
        // cout << "distances_.rows(), distances_.cols():" << endl << distances_.rows() << ',' << distances_.cols() << endl << endl;
        // cout << "indices_.rows(), indices_.cols():" << endl << indices_.rows() << ',' << indices_.cols() << endl << endl;
        // cout << "reference_points:" << endl << reference_points << endl << endl;
        // cout << "points:" << endl << points << endl << endl;
        MatrixXd points_pair_a(1,2), points_pair_b(1,2);
        int nn_index = 0;

        for(int i = 0; i < distances_.size(); i++) {
            if(distances_(nn_index) < distance_threshold) {
                push_back_(points_pair_a, points.block<2,1>(0,nn_index), nn_index);
                push_back_(points_pair_b, reference_points.block<2,1>(0,indices_(nn_index)), nn_index);
                nn_index++;
            }
        }
        // cout << "points_pair_a:" << endl << points_pair_a << endl << endl;
        // cout << "points_pair_b:" << endl << points_pair_b << endl << endl;
        if(verbose) cout << "number of pairs found:" << points_pair_a.rows() << endl;
        if(points_pair_a.rows() < point_pairs_threshold) {
            if(verbose) cout << "No better solution can be found (very few point pairs)!" << endl;
            break;
        }
        double *xytheta = point_based_matching(points_pair_a, points_pair_b);
        // for(int i = 0; i < 3; i++) {
        //     cout << "xytheta[" << i << "]:" << endl << xytheta[i] << endl << endl;
        // }
        
        if(xytheta != NULL) {
            if(verbose) {
                cout << "Rotation: " << RAD2DEG(xytheta[2]) << "degrees" << endl;
                cout << "Translation: " << xytheta[0] << ", " << xytheta[1] << endl;
            }
        }
        else {
            if(verbose) cout << "No better solution can be found!" << endl;
            break;
        }
        Matrix2d rot;
        rot << cos(xytheta[2]), -sin(xytheta[2]),
                sin(xytheta[2]), cos(xytheta[2]);
        MatrixXd aligned_points = rot * points;
        for(int j = 0; j < aligned_points.cols(); j++) {
            aligned_points.block<2,1>(0,j) += Vector2d(xytheta[0],xytheta[1]);
        }

        points = aligned_points;
        
        if( (abs(xytheta[2]) < convergence_rotation_threshold)
            && (abs(xytheta[0]) < convergence_translation_threshold)
            && (abs(xytheta[1]) < convergence_translation_threshold) ) {
            if(verbose) cout << "Converged!" << endl;
            break;
        }
    }
    return points;
}

void icp_lidar_ros::run() {
    MatrixXd aligned_points;
    sensor_msgs::LaserScan result_laserscan;
    ros::Rate rate(25);
    while(ros::ok()) {
        if(scan1 && scan2) {
            cout << "Publishing aligned scan topic (/scan)"
            int n = (angle_max_-angle_min_) / angle_increment_;
            aligned_points = icp(reference_points, points_to_be_aligned, 100, 0.3, 1e-3, 1e-4, 10, false);
            std::vector<float> ranges(n);
            for(int i = 0; i < aligned_points.cols() ; i++) {
                double angle = atan2(aligned_points(1,i), aligned_points(0,i));
                int index = (-angle_min_ - angle) / angle_increment_;
                ranges[index] = sqrt(pow(aligned_points(0,i), 2) + pow(aligned_points(1,i), 2));
            }
            result_laserscan.header.frame_id = "laser1";
            result_laserscan.angle_min = angle_min_;
            result_laserscan.angle_max = angle_max_;
            result_laserscan.angle_increment = angle_increment_;
            result_laserscan.range_min = range_min_;
            result_laserscan.range_max = range_max_;
            result_laserscan.ranges = ranges;
            aligned_scan_pub_.publish(result_laserscan); // scan publish
            scan1 = false; scan2 = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

Vector2d icp_lidar_ros::Split_(string input, char delimiter) {
    Vector2d answer;
    stringstream ss(input);
    string temp;

    for(int i = 0; getline(ss, temp, delimiter); i++) {
        answer(i) = stod(temp);
    }
    return answer;
}

void icp_lidar_ros::push_back_(MatrixXd& m, Vector2d&& values, std::size_t row) {
    if(row >= m.rows()) {
        m.conservativeResize(row + 1, Eigen::NoChange);
    }
    m.row(row) = values;
}