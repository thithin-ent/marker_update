#include "marker_update/marker_update.h"


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "marker_update");
    ros::NodeHandle nh;

    marker_update marker_update;
    ros::spin();
    return 0;
}