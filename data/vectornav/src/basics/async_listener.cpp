#include <stdio.h>
#include <unistd.h>
#include "vectornav/vectornav.h"
#include "ros/ros.h"

/* Change the connection settings to your configuration. */

const char* const COM_PORT = "//dev//ttyU1B0";
const int BAUD_RATE = 115200;

int main()
{
    Vn200 vn200;
    int i;

    vn200_connect(&vn200, COM_PORT, BAUD_RATE);

    /* Pause to ensure we have received the first asynchronous data record from the sensor. */
    sleep(1);

    for (i = 0; i < 10; i++) {
        Vn200CompositeData data;
        vn200_getCurrentAsyncData(&vn200, &data);
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
            data.ypr.yaw,
            data.ypr.pitch,
            data.ypr.roll,
            data.latitudeLongitudeAltitude.c0,
            data.latitudeLongitudeAltitude.c1,
            data.latitudeLongitudeAltitude.c2,
            data.velocity.c0,
            data.velocity.c1,
            data.velocity.c2);
        printf("\n\n");
        sleep(1);
    }
    
    vn200_disconnect(&vn200);

    return 0;
}
