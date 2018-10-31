#include "base_driver.h"

int main(int argc, char ** argv)
{
    ros::init( argc, argv, "base_driver_node");
    kaqi_driver::BaseDriver chasis;
    ros::spin();
    return 0;
}
