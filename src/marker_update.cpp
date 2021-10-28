#include "marker_update/marker_update.h"

MarkerUpdate::MarkerUpdate(const int &landmark)
{
    landmark_N_ = landmark;
    X_.resize(3 + landmark_N_ * 3);
    X_ << VectorXd::Zero(3), VectorXd::Zero(landmark_N_ * 3);
    P_.resize(3 + landmark_N_ * 3, 3 + landmark_N_ * 3);
    P_ << MatrixXd::Zero(3, 3), MatrixXd::Zero(3, landmark_N_ * 3),
        MatrixXd::Zero(landmark_N_ * 3, 3), 50000 * MatrixXd::Identity(landmark_N_ * 3, landmark_N_ * 3);
    Q_.resize(3, 3);
    Q_ << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    tf_br.resize(landmark_N_);
    //std::cout << "set_sstart" << std::endl;
    //pose_sub = nh.subscribe("pose", 1, &MarkerUpdate::posecallback, this);
    aruco_sub = nh.subscribe("fiducial_transforms", 1, &MarkerUpdate::markercallback, this);
}

void MarkerUpdate::correction(const VectorXd &Z, const MatrixXd &H, const VectorXd &z_hat)
{
    //std::cout << "----X_--------" << std::endl;
    //std::cout << X_ << std::endl;
    //std::cout << "----P_--------" << std::endl;
    //std::cout << P_ << std::endl;
    MatrixXd K(3 + landmark_N_ * 3, 3);
    K = P_ * H.transpose() * (H * P_ * H.transpose() + Q_).inverse();
    X_ = X_ + K * (Z - z_hat);
    P_ = (MatrixXd::Identity(3 + landmark_N_ * 3, 3 + landmark_N_ * 3) - K * H) * P_;
}

void MarkerUpdate::posecallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &data)
{

    X_(0) = data->pose.pose.position.x;
    X_(1) = data->pose.pose.position.y;
    X_(2) = tf::getYaw(data->pose.pose.orientation);
    P_(0, 0) = data->pose.covariance[0];
    P_(0, 1) = data->pose.covariance[1];
    P_(1, 0) = data->pose.covariance[6];
    P_(1, 1) = data->pose.covariance[7];
    P_(2, 2) = data->pose.covariance[35];
    //std::cout << "-------------" << std::endl;
    //std::cout << P_ << std::endl;
    //std::cout << "-------------" << std::endl;
}

void MarkerUpdate::markercallback(const indoor_2d_nav::FiducialTransformArray_i2n::ConstPtr &data)
{

    int marker_index = data->detected_count;
    for (int i = 0; i < marker_index; i++)
    {
        double error = data->transforms[i].image_error;
        //Q_ << error, 0, 0, 0, error, 0, 0, 0, error;
        int marker_id = data->transforms[i].fiducial_id;
        VectorXd Z(3);
        tf::Quaternion T_quaternion(data->transforms[i].transform.rotation.x, data->transforms[i].transform.rotation.y,
                                    data->transforms[i].transform.rotation.z, data->transforms[i].transform.rotation.w);

        double roll, pitch, yaw;
        tf::Matrix3x3(T_quaternion).getRPY(roll, pitch, yaw);
        Z << data->transforms[i].transform.translation.z, -data->transforms[i].transform.translation.x, -pitch;

        MatrixXd low_H(3, 6);
        low_H << -cos(X_(2)), -sin(X_(2)), sin(X_(2)) * X_(3 + marker_id * 3) + cos(X_(2)) * X_(4 + marker_id * 3) + sin(X_(2)) * X_(0) - cos(X_(2)) * X_(1), cos(X_(2)), sin(X_(2)), 0,
            sin(X_(2)), -cos(X_(2)), -cos(X_(2)) * X_(3 + marker_id * 3) - sin(X_(2)) * X_(4 + marker_id * 3) + cos(X_(2)) * X_(0) + sin(X_(2)) * X_(1), -sin(X_(2)), cos(X_(2)), 0,
            0, 0, -1, 0, 0, 1;

        MatrixXd F(6, 3 + landmark_N_ * 3);
        F << MatrixXd::Identity(3, 3), MatrixXd::Zero(3, landmark_N_ * 3),
            MatrixXd::Zero(3, 3 + marker_id * 3), MatrixXd::Identity(3, 3), MatrixXd::Zero(3, 3 * landmark_N_ - 3 * marker_id - 3);

        std::cout << "----z_hat-------" << std::endl;

        VectorXd z_hat(3);
        z_hat(0) = cos(X_(2)) * X_(3 + marker_id * 3) + sin(X_(2)) * X_(4 + marker_id * 3) - cos(X_(2)) * X_(0) - sin(X_(2)) * X_(1);
        z_hat(1) = -sin(X_(2)) * X_(3 + marker_id * 3) + cos(X_(2)) * X_(4 + marker_id * 3) + sin(X_(2)) * X_(0) - cos(X_(2)) * X_(1);
        if (X_(5 + marker_id * 3) >= 0)
        {
            if (abs(X_(5 + marker_id * 3) - X_(2)) > abs(X_(5 + marker_id * 3) - 2 * M_PI - X_(2)))
                z_hat(2) = X_(5 + marker_id * 3) - 2 * M_PI - X_(2);
            else
                z_hat(2) = X_(5 + marker_id * 3) - X_(2);
        }
        else
        {
            if (abs(X_(5 + marker_id * 3) - X_(2)) > abs(X_(5 + marker_id * 3) + 2 * M_PI - X_(2)))
                z_hat(2) = X_(5 + marker_id * 3) + 2 * M_PI - X_(2);
            else
                z_hat(2) = X_(5 + marker_id * 3) - X_(2);
        }
        std::cout << z_hat << std::endl;

        correction(Z, low_H * F, z_hat);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(X_(3 + marker_id * 3), X_(4 + marker_id * 3), 0.0));
        tf::Quaternion q;
        q.setRPY(M_PI / 2, 0, -M_PI / 2 + X_(5 + marker_id * 3));
        transform.setRotation(q);
        std::string str("markerupdate_");
        tf_br[marker_id].sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", str + std::to_string(marker_id)));

        transform.setOrigin(tf::Vector3(z_hat(0), z_hat(1), 0.0));
        q.setRPY(M_PI / 2, 0, -M_PI / 2 + z_hat(2));
        transform.setRotation(q);
        std::string str1("test_");
        tf_br[marker_id].sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", str1 + std::to_string(marker_id)));
    }
}

void MarkerUpdate::set_robot_x(const double &robot_x, const double &robot_y, const double &robot_yaw){
    X_(0) = robot_x;
    X_(1) = robot_y;
    X_(2) = robot_yaw;
}

int MarkerUpdate::get_markers_number()
{
    return landmark_N_;
}

double MarkerUpdate::get_markers_x(const int &number)
{
    return X_(3 + number * 3);
}

double MarkerUpdate::get_markers_y(const int &number)
{
    return X_(4 + number * 3);
}

double MarkerUpdate::get_markers_head(const int &number)
{
    return X_(5 + number * 3);
}
