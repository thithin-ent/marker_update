#include "marker_update/marker_update.h"

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "marker_update");
    ros::NodeHandle nh;
    int randmark;
    nh.getParam("landmark_N" , randmark);

    marker_update marker_update(randmark);
    ros::spin();

    std::string path = ros::package::getPath("marker_update");
    std::ofstream w_file(path + "/marker/marker.txt");
    if (w_file.is_open())
    {
        for (int marker_id = 0; marker_id < marker_update.randmark_N; marker_id++)
        {
            tf::Quaternion q;
            q.setRPY(M_PI / 2, 0, -M_PI / 2 + marker_update.X_(5 + marker_id * 3));
            w_file << std::to_string(marker_id) << " " << std::to_string(marker_update.X_(3 + marker_id * 3)) << " " << std::to_string(marker_update.X_(4 + marker_id * 3)) << " 0.0 "
                   << std::to_string(q.x()) << " " << std::to_string(q.y()) << " " << std::to_string(q.z()) << " " << std::to_string(q.w()) << endl;
        }
    }

    return 0;
}
