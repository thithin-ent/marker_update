#include "marker_update/marker_update.h"

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "marker_update");
    ros::NodeHandle nh;
    int landmark;
    nh.getParam("landmark_N" , landmark);

    MarkerUpdate *marker_update = new MarkerUpdate(landmark);
    ros::spin();

    std::string path = ros::package::getPath("marker_update");
    std::ofstream w_file(path + "/marker/marker.txt");
    if (w_file.is_open())
    {
        for (int marker_id = 0; marker_id < marker_update->get_markers_number(); marker_id++)
        {
            tf::Quaternion q;
            q.setRPY(0, 0, marker_update->get_markers_head(marker_id));
            w_file << std::to_string(marker_id) << " " << std::to_string(marker_update->get_markers_x(marker_id)) << " " << std::to_string(marker_update->get_markers_y(marker_id)) << " 0.0 "
                   << std::to_string(q.x()) << " " << std::to_string(q.y()) << " " << std::to_string(q.z()) << " " << std::to_string(q.w()) << endl;
        }
    }

    return 0;
}
