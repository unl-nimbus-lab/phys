#include <stdio.h>
#include <unistd.h>
#include "vectornav/vectornav.h"
#include "ros/ros.h"

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 115200;

void asyncDataListener(Vn200* sender, Vn200CompositeData* data);

int main()
{
    Vn200 vn200;

    vn200_connect(&vn200, COM_PORT, BAUD_RATE);

    vn200_registerAsyncDataReceivedListener(&vn200, &asyncDataListener);

    sleep(10);

    vn200_unregisterAsyncDataReceivedListener(&vn200, &asyncDataListener);
    
    vn200_disconnect(&vn200);

    return 0;
}
void asyncDataListener(Vn200* sender, Vn200CompositeData* data)
{
    ROS_INFO("INS Solution:\n"
        "  YPR.Yaw:                %+#7.2f\n"
        "  YPR.Pitch:              %+#7.2f\n"
        "  YPR.Roll:               %+#7.2f\n"
        "  LLA.Lattitude:          %+#7.2f\n"
        "  LLA.Longitude:          %+#7.2f\n"
        "  LLA.Altitude:           %+#7.2f\n"
        "  Velocity.North:         %+#7.2f\n"
        "  Velocity.East:          %+#7.2f\n"
        "  Velocity.Down:          %+#7.2f\n",
        data->ypr.yaw,
        data->ypr.pitch,
        data->ypr.roll,
        data->latitudeLongitudeAltitude.c0,
        data->latitudeLongitudeAltitude.c1,
        data->latitudeLongitudeAltitude.c2,
        data->velocity.c0,
        data->velocity.c1,
        data->velocity.c2);
    ROS_INFO("\n\n");
}
