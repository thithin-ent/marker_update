#include "marker_update/marker_update.h"

marker_update::marker_update()
{
  X_.resize(3 + randmark_N * 3);
  X_ << VectorXd::Zero(3), VectorXd::Zero(randmark_N * 3);
  P_.resize(3 + randmark_N * 3, 3 + randmark_N * 3);
  P_ << MatrixXd::Zero(3, 3), MatrixXd::Zero(3, randmark_N * 3),
      MatrixXd::Zero(randmark_N * 3, 3), 50000 * MatrixXd::Identity(randmark_N * 3, randmark_N * 3);
  Q_.resize(3, 3);
  Q_ << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
  T_B_C << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  std::cout << "set_sstart" << std::endl;
  pose_sub = nh.subscribe("pose", 1, &marker_update::posecallback, this);
  aruco_sub = nh.subscribe("fiducial_transforms", 1, &marker_update::markercallback, this);
}

void marker_update::correction(const VectorXd &Z, const MatrixXd &H, const VectorXd &z_hat)
{
  std::cout << "----X_--------" << std::endl;
  std::cout << X_ << std::endl;
  std::cout << "----P_--------" << std::endl;
  std::cout << P_ << std::endl;
  MatrixXd K(3 + randmark_N * 3, 3);
  K = P_ * H.transpose() * (H * P_ * H.transpose() + Q_).inverse();
  X_ = X_ + K * (Z - z_hat);
  P_ = (MatrixXd::Identity(3 + randmark_N * 3, 3 + randmark_N * 3) - K * H) * P_;
}

void marker_update::posecallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &data)
{

  X_(0) = data->pose.pose.position.x;
  X_(1) = data->pose.pose.position.y;
  X_(2) = tf::getYaw(data->pose.pose.orientation);
}

void marker_update::markercallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &data)
{

  int marker_index = data->detected_count;
  for (int i = 0; i < marker_index; i++)
  {
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

    MatrixXd F(6, 3 + randmark_N * 3);
    F << MatrixXd::Identity(3, 3), MatrixXd::Zero(3, randmark_N * 3),
        MatrixXd::Zero(3, 3 + marker_id * 3), MatrixXd::Identity(3, 3), MatrixXd::Zero(3, 3 * randmark_N - 3 * marker_id - 3);

    std::cout << "----z_hat-------" << std::endl;

    Vector3d z_hat;
    z_hat << cos(X_(2)) * X_(3 + marker_id * 3) + sin(X_(2)) * X_(4 + marker_id * 3) - cos(X_(2)) * X_(0) - sin(X_(2)) * X_(1),
        -sin(X_(2)) * X_(3 + marker_id * 3) + cos(X_(2)) * X_(4 + marker_id * 3) + sin(X_(2)) * X_(0) - cos(X_(2)) * X_(1),
        X_(5 + marker_id * 3) - X_(2);
    std::cout << z_hat << std::endl;

    correction(Z, low_H * F, z_hat);
  }
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(X_(3), X_(4), 0.0));
    tf::Quaternion q;
    q.setRPY(M_PI/2, 0, -M_PI/2 + X_(5));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "markerupdate_1"));
}