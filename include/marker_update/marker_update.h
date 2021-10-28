#ifndef MARKER_UPDATE
#define MARKER_UPDATE

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include <ros/package.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <indoor_2d_nav/FiducialTransformArray_i2n.h>

using namespace std;
using namespace Eigen;

class MarkerUpdate
{
public:
    MarkerUpdate(const int &landmark);
    void correction(const VectorXd &Z, const MatrixXd &H, const VectorXd &z_hat);
    void posecallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &data);
    void markercallback(const indoor_2d_nav::FiducialTransformArray_i2n::ConstPtr &data);
    void set_robot_x(const double &robot_x, const double &robot_y, const double &robot_yaw);
    int get_markers_number();
    double get_markers_x(const int &number);
    double get_markers_y(const int &number);
    double get_markers_head(const int &number);

private:
    int landmark_N_;
    VectorXd X_;
    MatrixXd P_;
    MatrixXd Q_;
    std::vector<tf::TransformBroadcaster> tf_br;
    ros::NodeHandle nh;
    //ros::Publisher pub_;
    ros::Subscriber pose_sub;
    ros::Subscriber aruco_sub;
};
#endif