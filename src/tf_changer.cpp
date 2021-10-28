#include "tf_changer/tf_changer.h"
#include "std_msgs/Float32MultiArray.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tf_changer");
    ros::NodeHandle nh;
    ros::Publisher pub_pos = nh.advertise<std_msgs::Float32MultiArray>("robot_position", 10);

    tf::TransformListener listener;

    ros::Rate rate(10.0);

    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/base_footprint", "/map", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_footprint", "/map", ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        tf::Quaternion q = transform.getRotation();
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::vector<float> msg_data = {static_cast<float>(transform.getOrigin().x()), static_cast<float>(transform.getOrigin().y()), static_cast<float>(yaw)};

        std_msgs::Float32MultiArray msg;
        msg.data = msg_data;

        pub_pos.publish(msg);

        rate.sleep();
    }
    ros::spin();

    return 0;
}
