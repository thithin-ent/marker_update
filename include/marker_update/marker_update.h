#ifndef MARKER_UPDATE
#define MARKER_UPDATE
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;

class marker_update
{
public:
    int randmark_N = 2;
    VectorXd X_;
    MatrixXd P_;
    MatrixXd Q_;
    Matrix3d T_B_C;
    marker_update();
    void correction(const VectorXd &Z, const MatrixXd &H, const VectorXd &z_hat);
    void posecallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &data);
    void markercallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &data);

private:
    ros::NodeHandle nh;
    //ros::Publisher pub_;
    ros::Subscriber pose_sub;
    ros::Subscriber aruco_sub;
};

#endif