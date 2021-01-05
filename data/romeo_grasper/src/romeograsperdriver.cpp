#include "romeo_grasper/romeograsper.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "romeo_grasper");
    //ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    RomeoGrasper romeo_grasper = RomeoGrasper();
    romeo_grasper.exit();
    return 0;
}
