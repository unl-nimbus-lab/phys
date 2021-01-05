#include <stdint.h>
extern "C" {
#include "proxyTypes.h"
#include "car_controller.h"                  /* Model's header file */
#include "rtwtypes.h"
//#include "multiword_types.h"
#include "ext_work.h"                  /* External mode header file */
}

#define EXT_MODE_PORT 17755

//#pragma pack(1)

struct Command
{
	int8_t power; //(-100, 100)
	int8_t direction;//(-100, 100)
	int8_t ledFrontRight;//(0, 100)
	int8_t ledFrontLeft;
	int8_t ledRearRight;
	int8_t ledRearLeft;
	int8_t brake;//0 or 1
	uint8_t autonomous; //0(manual) - 1(autonomous)
//	uint8_t targetType; //0(yellow) - 1(green) - 2(red)
//	int16_t centerX;  // width pixels
//	int16_t centerY; //height: use future pixels
//	uint8_t radius;  //pixels. real radius is 0.085m
};

struct AutoCommand
{
	int16_t posX;
	int16_t posY;
	uint8_t vel;
};

struct TviconProxy
{
    TState state;
    char  name[32];
    float dAttitude[3];
    float dPosition[3];
    unsigned int uiHeartbeat;

};

struct TviconData
{
    int structNumber;
    TviconProxy data[1];
};

void *receive_thread(void *argument);
void *receive_thread_auto(void *argument);

void initializeMatlabModel();
void stepExternalMode();

void initSockets();
bool closeConnection();
