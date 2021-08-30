#include "marker_update/marker_update.h"

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "marker_update");
    ros::NodeHandle nh;

    marker_update marker_update;
    ros::spin();

    std::string path = ros::package::getPath("marker_update");
    std::ofstream w_file(path + "/marker/marker.txt");
    if (w_file.is_open())
    {
        for (int i = 0; i < marker_update.randmark_N; i++)
        {
            tf::Quaternion q;
            q.setRPY(M_PI / 2, 0, -M_PI / 2 + marker_update.X_(5 + i * 3));
            w_file << std::to_string(i) << " " << std::to_string(marker_update.X_(3 + i * 3)) << " " << std::to_string(marker_update.X_(4 + i * 3)) << " 0.0 "
            << std::to_string(q.x()) << " " << std::to_string(q.y()) << " " << std::to_string(q.z()) << " " << std::to_string(q.w()) << endl;
        }
    }
    else {
        std::cout<< path + "marker/marker.txt" << std::endl;
    }

    return 0;
}
