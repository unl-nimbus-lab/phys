//gthread
// #include <glib.h>

// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

// ulink define
#define HEADER 0xb5
#define toUint(X)    *(uint16_t*)(&X)
#define toSint(X)    *(int16_t*)(&X)
#define toFlt(X)    *(float*)(&X)
#define toLong(X)    *(int32_t*)(&X)

inline static int sendbyte(int s, uint8_t buff) 
{  
  return write(s, &buff, 1);
}

//ulink massage id define
#define ATTITUDE        1 //int 3
#define RAW_IMU         2 //int 6
#define RAW_BARO        3 //float 3
#define RAW_RADIO       4 //int 5
#define MOTOR_OUT       5 //int 4
#define ALTITUDE        6 //float 3
#define ACC_AXIS        7 //float 3
#define V_VAL           8 //float 2
#define CNT_VAL         9 //float 3 cnt roll,pitch,yaw
#define STATUS_CHK      10 //int 1
#define GPS             11 //long 2
#define DATA_REQUEST    12 //chr 1
#define CMD_TAKEOFF     13 //chr 1
#define UNKNOWN_TYPE     14 //unknown
#define UNKNOWN_FLOAT    15
#define CONT_COMPASS     16  //unused
#define CONT_PID         17  //unused
#define NAV_DATA         18
#define COMPASS          19
#define GPS_RAW_FIX      20
#define ACK              21
#define POSE_DEMAND      22
#define VISION_DATA      23
#define SONAR_DATA       24
#define FLOW_DATA        25
#define MSF_DATA         26
#define DESIRE_NAV       27

#define GCONFIG     100 
#define PID_RPY1    101 
#define PID_RPY2    102 
#define PID_ALT     103 
#define PID_NAV     104 
#define PID_SPD     105 
#define NAV_INER    106 
#define MODE_CFG    107 

using std::string;
using namespace std;

struct timeval tv;      ///< System time

int baud = 115200;                 ///< The serial baud rate

// Settings
int serial_compid = 0;
std::string port = "/dev/ttyACM0";              ///< The serial port name, e.g. /dev/ttyUSB0
bool silent = false;              ///< Wether console output should be enabled
bool verbose = false;             ///< Enable verbose output
bool debug = false;               ///< Enable debug functions and output
bool pc2serial = true;        ///< Enable PC to serial push mode (send more stuff from pc over serial)
int fd;


std::string frame_id("fcu");

//ulnik function
inline static void send4byte(int s,int32_t n) {
  char buff[4];
  buff[0]= uint8_t((n<<24)>>24);
  buff[1]= uint8_t((n<<16)>>24);
  buff[2]= uint8_t((n<<8)>>24);
  buff[3]= uint8_t(n>>24);
	write(s,buff ,4);

}

inline static void send2byte(int s,int n) {
    char buff[2];
    buff[0]= uint8_t((n<<8)>>8);
    buff[1]= uint8_t(n>>8);
    write(s,buff ,2);


}
inline static uint16_t sum4byte(int32_t n) {
  return uint8_t(n>>24) + uint8_t((n<<8)>>24) + uint8_t((n<<16)>>24)+uint8_t((n<<24)>>24);
}
inline static uint16_t sum2byte(uint16_t n) {
  return uint8_t(n>>8) + uint8_t((n<<8)>>8);
}


// from asctec_hl_interface
inline void angle2quaternion(const double &roll, const double &pitch, const double &yaw, double *w, double *x,
                             double *y, double *z)
{
  double sR2, cR2, sP2, cP2, sY2, cY2;
  sincos(roll * 0.5, &sR2, &cR2);
  sincos(pitch * 0.5, &sP2, &cP2);
  sincos(yaw * 0.5, &sY2, &cY2);

  // TODO: change rotation order
  // this follows AscTec's pre- 2012 firmware rotation order: Rz*Rx*Ry
//  *w = cP2 * cR2 * cY2 - sP2 * sR2 * sY2;
//  *x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
//  *y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
//  *z = cP2 * cR2 * sY2 + cY2 * sP2 * sR2;

  // Rz*Ry*Rx for 2012 firmware on the:
  *w = cP2 * cR2 * cY2 + sP2 * sR2 * sY2;
  *x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
  *y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
  *z = cP2 * cR2 * sY2 - cY2 * sP2 * sR2;
}


/**
 *
 *
 * Returns the file descriptor on success or -1 on error.
 */

int open_port(std::string& port)
{
  int fd; /* File descriptor for the port */

  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C
  fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  {
    /* Could not open the port. */
    return (-1);
  }
  else
  {
    fcntl(fd, F_SETFL, 0);
  }

  return (fd);
}



bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
  //struct termios options;

  struct termios config;
  if (!isatty(fd))
  {
    fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
    return false;
  }
  if (tcgetattr(fd, &config) < 0)
  {
    fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
    return false;
  }
  //
  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  //
  config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
  //
  // Output flags - Turn off output processing
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  //
  config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

#ifdef OLCUC
  config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
  config.c_oflag &= ~ONOEOT;
#endif

  //
  // No line processing:
  // echo off, echo newline off, canonical mode off,
  // extended input processing off, signal chars off
  //
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  //
  // Turn off character processing
  // clear current char size mask, no parity checking,
  // no output processing, force 8 bit input
  //
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;
  //
  // One input byte is enough to return from read()
  // Inter-character timer off
  //
  config.c_cc[VMIN] = 1;
  config.c_cc[VTIME] = 10; // was 0

  // Get the current options for the port
  //tcgetattr(fd, &options);

  switch (baud)
  {
    case 1200:
      if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    case 1800:
      cfsetispeed(&config, B1800);
      cfsetospeed(&config, B1800);
      break;
    case 9600:
      cfsetispeed(&config, B9600);
      cfsetospeed(&config, B9600);
      break;
    case 19200:
      cfsetispeed(&config, B19200);
      cfsetospeed(&config, B19200);
      break;
    case 38400:
      if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    case 57600:
      if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    case 115200:
      if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;

      // These two non-standard (by the 70'ties ) rates are fully supported on
      // current Debian and Mac OS versions (tested since 2010).
    case 460800:
      if (cfsetispeed(&config, 460800) < 0 || cfsetospeed(&config, 460800) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    case 921600:
      if (cfsetispeed(&config, 921600) < 0 || cfsetospeed(&config, 921600) < 0)
      {
        fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
        return false;
      }
      break;
    default:
      fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
      return false;

      break;
  }

  //
  // Finally, apply the configuration
  //
  if (tcsetattr(fd, TCSAFLUSH, &config) < 0)
  {
    fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
    return false;
  }
  return true;
}

void close_port(int fd)
{
  close(fd);
}

uint8_t st = 0,_lenp,_msgid,_pay[512],_idp,_c[2];
uint16_t _sum_pay,_csum;

inline void packet_decode(void);
inline void serial_handle(void* serial_ptr)
{
  int fd = *((int*)serial_ptr);
    uint8_t cp;
  uint8_t msgReceived = false;
    if (read(fd, &cp, 1) > 0)
    {
      // Check if a message could be decoded, return the message in case yes
    unsigned char dd = cp;
    switch (st) {
          case 0:
            _sum_pay=0;
            if (dd == HEADER) {
              _sum_pay+=dd;
              st++;
            }
            break;
          case 1 :
            _lenp = dd;
            _sum_pay+=dd;
            st++;
            break;
          case 2 :
            _msgid = dd;
            _sum_pay+=dd;
            _idp = 0;
            st++;
            break;
          case 3 :
            _pay[_idp++] = dd;
            _sum_pay+=dd;
            if (_idp >=_lenp) st++;
            break;
          case 4 :
            _c[0] = dd;
            st++;
            break;
          case 5 :
            _c[1] = dd;
            _csum = toUint(_c);
            st = 0;
            if (_sum_pay == _csum) {
              packet_decode();
              msgReceived = true;
            }
            break;
        }
    }
}

inline void packet_decode(void) 
{
  ros::Time t_now=ros::Time::now();


  if (_msgid == ATTITUDE) { //int3
    int16_t d[3];
    d[0] = toSint(_pay[0]);
    d[1] = toSint(_pay[2]);
    d[2] = toSint(_pay[4]);
    
    NavMsg.tm   = 1000000*(t_now.sec-time_start.sec)+(t_now.nsec-time_start.nsec)/1000;
    NavMsg.rotX = d[0]*0.0572957;
    NavMsg.rotY = d[1]*0.0572957;
    NavMsg.rotZ = d[2]*0.0572957;

    tf::Quaternion q;
    q.setRPY(d[0]*0.001,d[1]*0.001,d[2]*0.001);  //roll pitch yaw   (addition for use in rviz +p/2)

    imuMsg.header.stamp = t_now; //-self.time_start;
    imuMsg.orientation.x=q[0];
    imuMsg.orientation.y=q[1];
    imuMsg.orientation.z=q[2];
    imuMsg.orientation.w=q[3];
                
    poseMsg.pose.pose.orientation.x=imuMsg.orientation.x;
    poseMsg.pose.pose.orientation.y=imuMsg.orientation.y;
    poseMsg.pose.pose.orientation.z=imuMsg.orientation.z;
    poseMsg.pose.pose.orientation.w=imuMsg.orientation.w;
  }
  else if(_msgid == RAW_IMU) { //int6
    int16_t d[6];
    d[0] = toSint(_pay[0]);
    d[1] = toSint(_pay[2]);
    d[2] = toSint(_pay[4]);
    d[3] = toSint(_pay[6]);
    d[4] = toSint(_pay[8]);
    d[5] = toSint(_pay[10]);
    
    imuMsg.header.stamp = t_now;
    imuMsg.linear_acceleration.x = d[0]*9.80655/1000.0;              
    imuMsg.linear_acceleration.y = d[1]*9.80655/1000.0;
    imuMsg.linear_acceleration.z = d[2]*9.80655/1000.0;
    imuMsg.angular_velocity.x = d[3]/1000.0;       
    imuMsg.angular_velocity.y = d[4]/1000.0;
    imuMsg.angular_velocity.z = d[5]/1000.0;
    // compassMsg.header.stamp = t_now; //-self.time_start;
                
    // compassMsg.magnetic_field.x = d[6];
    // compassMsg.magnetic_field.y = d[7];
    // self.compassMsg.magnetic_field.z = d[8];
 
    NavMsg.ax = d[0]/1000.00;
    NavMsg.ay = d[1]/1000.00;
    NavMsg.az = d[2]/1000.00;

    if(t_now - msf_data.header.stamp < ros::Duration(0.08) ) {  // update fake navdata field if lastest data from odom is come (80ms~)
      NavMsg.vx=msf_data.twist.twist.linear.x*1000;
      NavMsg.vy=msf_data.twist.twist.linear.y*1000;
      NavMsg.vz=msf_data.twist.twist.linear.z*1000;

    }
    
  }
  else if(_msgid == V_VAL) { //int2
    int16_t d[2];
    d[0] = toSint(_pay[0]);
    d[1] = toSint(_pay[2]);
    
    pose_navMsg.twist.twist.linear.x = d[0]*0.01;
      pose_navMsg.twist.twist.linear.y = d[1]*0.01;
  }
  else if(_msgid == RAW_BARO) { //float3
    float d[3];
    d[0] = toFlt(_pay[0]);
    d[1] = toFlt(_pay[4]);
    d[2] = toFlt(_pay[8]);
  }
  else if(_msgid == RAW_RADIO) { //int6
    int16_t d[6];
    d[0] = toSint(_pay[0]);
    d[1] = toSint(_pay[2]);
    d[2] = toSint(_pay[4]);
    d[3] = toSint(_pay[6]);
    d[4] = toSint(_pay[8]);
    d[5] = toSint(_pay[10]);
    
    remoteMsg.header.stamp = t_now; //-self.time_start
    remoteMsg.axes.clear();
    remoteMsg.axes.push_back(d[0]);
    remoteMsg.axes.push_back(d[1]);
    remoteMsg.axes.push_back(d[2]);
    remoteMsg.axes.push_back(d[3]);
    remoteMsg.axes.push_back(d[4]);
    remoteMsg.axes.push_back(d[5]);

  }
  else if(_msgid == MOTOR_OUT) { //int4
    int16_t d[4];
    d[0] = toSint(_pay[0]);
    d[1] = toSint(_pay[2]);
    d[2] = toSint(_pay[4]);
    d[3] = toSint(_pay[6]);
  }
  else if(_msgid == CNT_VAL) { //float4
    float d[4];
    d[0] = toFlt(_pay[0]);
    d[1] = toFlt(_pay[4]);
    d[2] = toFlt(_pay[8]);
    d[2] = toFlt(_pay[12]);
    
  }
  else if(_msgid == STATUS_CHK) { //int4
    int16_t d[4];
    d[0] = toSint(_pay[0]);
    d[1] = toSint(_pay[2]);
    d[2] = toSint(_pay[4]);
    d[3] = toSint(_pay[6]);
    
    printf("NUM SEN : %d  --ARM : %d  --MODE : %d  --RC : %d\n",d[0],d[1],d[2],d[3]);
  }
  else if(_msgid == UNKNOWN_TYPE) { // depend on len
  }
  else if(_msgid == UNKNOWN_FLOAT) { // depend on len
  }
  else if(_msgid == GPS) { //float2 int2
    float d[2]; 
    int16_t d2[2];
    
    d[0] = toFlt(_pay[0]);
    d[1] = toFlt(_pay[4]);
    d2[0] = toSint(_pay[8]);
    d2[1] = toSint(_pay[10]);
    
    poseMsg.header.stamp = t_now; //-self.time_start
        poseMsg.pose.pose.position.x = d[1]/100.00;  //to m
        poseMsg.pose.pose.position.y = d[0]/100.00;
        //ros UTM  y is northing , x is easting
        navvelMsg.header.stamp = t_now; //-self.time_start
        navvelMsg.twist.twist.linear.x = d2[0]/100.00;
        navvelMsg.twist.twist.linear.y = d2[1]/100.00;
        
        //printf("GPS Lat : %f Lon : %f\n",d[0],d[1]);
  }
  else if(_msgid == GPS_RAW_FIX) { //long2 uint2 long1 uint3
    int32_t d1[3];
    uint16_t d2[5];
    
    d1[0] = toLong(_pay[0]);
    d1[1] = toLong(_pay[4]);
    
    d2[0] = toUint(_pay[8]);
    d2[1] = toUint(_pay[10]);
    
    d1[2] = toLong(_pay[12]);
    
    d2[2] = toUint(_pay[16]);
    d2[3] = toUint(_pay[18]);
    d2[4] = toUint(_pay[20]);
    
    gpsrawMsg.header.stamp = t_now; //-self.time_start
      gpsrawMsg.latitude = d1[0]/10000000.00000000;  //deg
        gpsrawMsg.longitude = d1[1]/10000000.00000000;
        gpsrawMsg.status.status = (d2[2] == 0)? -1 : 1;
        gpsrawMsg.position_covariance[0] = d2[1]*0.01;
        gpsrawMsg.position_covariance[4] = d2[1]*0.01;
        gpsrawMsg.position_covariance[8] = d2[3]*0.01;
        
        poseMsg.pose.covariance[0]=poseMsg.pose.covariance[7]=d2[1]*0.01;
        poseMsg.pose.covariance[21]=poseMsg.pose.covariance[28]=poseMsg.pose.covariance[35]=1;
        navvelMsg.twist.covariance[0]=navvelMsg.twist.covariance[7]=d2[1]*0.01;
        navvelMsg.twist.covariance[14]=0.1;
                          
      gpsrawMsg.status.service = d2[0];
      gpsrawMsg.altitude = d1[2]/1000.00000000;
      gpsrawMsg.position_covariance_type = 2;
  }
  else if(_msgid == NAV_DATA) { //float2
    float d[2];
    d[0] = toFlt(_pay[0]);
    d[1] = toFlt(_pay[4]);
    
    pose_navMsg.header.stamp = t_now;
        pose_navMsg.pose.pose.position.x = d[0];  //to m
        pose_navMsg.pose.pose.position.y = d[1];
        //printf("Nav x: %f , Nav y: %f",d[0],d[1]);
  }
  else if(_msgid == ALTITUDE) { //float1 int1
    float d1 = toFlt(_pay[0]); 
    int16_t d2 = toSint(_pay[4]);
    
    pose_navMsg.pose.pose.position.z = d1;

        altMsg.header.stamp = t_now; //-self.time_start
        altMsg.pose.pose.position.z = d1;
        altMsg.twist.twist.linear.z = d2/100.000;  //to m/s
        // altMsg.pose.covariance=[0, 0, 0, 0, 0, 0,
        //           0, 0, 0, 0, 0, 0,
        //               0, 0, 0.1,0, 0, 0,
        //               0, 0, 0,0, 0, 0,
        //                 0, 0, 0, 0, 0, 0,
        //                   0, 0, 0, 0, 0, 0];
                          
        // altMsg.twist.covariance=[0, 0, 0, 0, 0, 0,
        //             0, 0,0, 0, 0, 0,
        //                 0, 0, 0.1,0, 0, 0,
        //                 0, 0, 0, 0, 0,0,
        //                 0, 0, 0, 0, 0, 0,
        //                 0, 0, 0, 0, 0, 0];
        navvelMsg.twist.twist.linear.z = d2/100.000;  //to m/s

        NavMsg.altd = int(d1*1000);  //report in mm
        
        //printf("ALT rel_ALT : %.2f vel_z : %4d",d1*100,d2*10));
  }
  else if(_msgid == CONT_COMPASS) { // int6
    int16_t d[6];
    d[0] = toSint(_pay[0]);
    d[1] = toSint(_pay[2]);
    d[2] = toSint(_pay[4]);
    d[3] = toSint(_pay[6]);
    d[4] = toSint(_pay[8]);
    d[5] = toSint(_pay[10]);
    
  }
  else if(_msgid == CONT_PID) { //uint12
    uint16_t d[12];
    d[0] = toUint(_pay[0]);
    d[1] = toUint(_pay[2]);
    d[2] = toUint(_pay[4]);
    d[3] = toUint(_pay[6]);
    d[4] = toUint(_pay[8]);
    d[5] = toUint(_pay[10]);
    d[6] = toUint(_pay[12]);
    d[7] = toUint(_pay[14]);
    d[8] = toUint(_pay[16]);
    d[9] = toUint(_pay[18]);
    d[10] = toUint(_pay[20]);
    d[11] = toUint(_pay[22]);
  }
  else if(_msgid == ACK) { //uint1
    uint16_t d= toUint(_pay[0]);
    bool ack[16];
    for (int i =0;i<16;i++) 
      ack[i] = bool(d  & (0x0001 << i));
    ackMsg.ACK_PENDING  =   ack[0];
        ackMsg.ACK_GCONFIG  =   ack[1];
        ackMsg.ACK_PID_RPY1 =   ack[2];
        ackMsg.ACK_PID_RPY2 =   ack[3];
        ackMsg.ACK_PID_ALT  =   ack[4];
        ackMsg.ACK_PID_NAV  =   ack[5];
        ackMsg.ACK_SPD      =   ack[6];
        ackMsg.ACK_NAV_INER =   ack[7];
        ackMsg.ACK_MODE     =   ack[8];
  }
  else if(_msgid == DESIRE_NAV) { //int7
    int16_t d[7];
    d[0] = toSint(_pay[0]);
    d[1] = toSint(_pay[2]);
    d[2] = toSint(_pay[4]);
    d[3] = toSint(_pay[6]);
    d[4] = toSint(_pay[8]);
    d[5] = toSint(_pay[10]);
    d[6] = toSint(_pay[12]);
    
    desire_navMSG.header.stamp = t_now; //-self.time_start
        desire_navMSG.pose.pose.position.x = d[0]*0.01;
        desire_navMSG.pose.pose.position.y = d[2]*0.01;
        desire_navMSG.pose.pose.position.z = d[4]*0.01;
        desire_navMSG.twist.twist.linear.x = d[1]*0.01;
        desire_navMSG.twist.twist.linear.y = d[3]*0.01;
        desire_navMSG.twist.twist.linear.z = d[5]*0.01;

    tf::Quaternion q;
    q.setRPY(0,0,d[6]*0.01);
        desire_navMSG.pose.pose.orientation.x = q[0];
        desire_navMSG.pose.pose.orientation.y = q[1];
        desire_navMSG.pose.pose.orientation.z = q[2];
        desire_navMSG.pose.pose.orientation.w = q[3];
                
  }
}
static void int_sent(int s,int n1,int n2,int n3) {
    uint8_t len = 6;
    uint8_t msg_id = ATTITUDE;
    uint16_t chk = HEADER + len + msg_id +sum2byte(n1)+sum2byte(n2)+sum2byte(n3);

    sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send2byte(s,n1); //data1
    send2byte(s,n2); //data2
    send2byte(s,n3); //data3
    send2byte(s,chk); //chk
}
static void flt_sent(int s,float _n1,float _n2,float _n3,float _n4) {
  
    int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);
    int32_t n3=toLong(_n3);
    int32_t n4=toLong(_n4);
    
    uint8_t len = 16;
    uint8_t msg_id = CNT_VAL;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2)+sum4byte(n3)+sum4byte(n4);
    
    sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1); //data1
    send4byte(s,n2); //data2
    send4byte(s,n3); //data3
    send4byte(s,n4); //data3
    send2byte(s,chk); //chk
}

static void send_cmd_takeoff(int s , int h) {
	
    uint8_t len = 1;
    uint8_t msg_id = CMD_TAKEOFF;  
    uint16_t chk = HEADER + len + msg_id +sum2byte(h);

    sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send2byte(s,h); //data1
    send2byte(s,chk); //chk
    
}

static void send_position_desire(int s,float _n1,float _n2,float _n3,float _n4) {
	int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);
    int32_t n3=toLong(_n3);
    int32_t n4=toLong(_n4);
    
    uint8_t len = 16;
    uint8_t msg_id = POSE_DEMAND;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2)+sum4byte(n3)+sum4byte(n4);

    sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1); //data1
    send4byte(s,n2); //data2
    send4byte(s,n3); //data3
    send4byte(s,n4); //data3
    send2byte(s,chk); //chk
}
static void send_flow(int s,float _n1,float _n2,uint16_t n3) {
	int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);
	
	uint8_t len = 10;
    uint8_t msg_id = POSE_DEMAND;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2)+sum2byte(n3);
    
    sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1); //data1
    send4byte(s,n2); //data2
    send2byte(s,n3); //data3
    send2byte(s,chk); //chk
}
static void send_motor_desire(int s,float _n1,float _n2,float _n3,float _n4) {
	int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);
    int32_t n3=toLong(_n3);
    int32_t n4=toLong(_n4);
    
    uint8_t len = 16;
    uint8_t msg_id = MOTOR_OUT;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2)+sum4byte(n3)+sum4byte(n4);
	
	sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1); //data1
    send4byte(s,n2); //data2
    send4byte(s,n3); //data3
    send4byte(s,n4); //data3
    send2byte(s,chk); //chk
}
static void send_pid1_param(int s,float _n1,float _n2,float _n3,float _n4,float _n5,float _n6) {
	int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);
    int32_t n3=toLong(_n3);
    int32_t n4=toLong(_n4);
    int32_t n5=toLong(_n5);
    int32_t n6=toLong(_n6);
    
    uint8_t len = 24;
    uint8_t msg_id = PID_RPY1;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2)+sum4byte(n3)+sum4byte(n4)+sum4byte(n5)+sum4byte(n6);
	
	sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1); 
    send4byte(s,n2);
    send4byte(s,n3);
    send4byte(s,n4); 
    send4byte(s,n5); 
    send4byte(s,n6); 
    send2byte(s,chk); //chk
}
static void send_pid2_param(int s,float _n1,float _n2,float _n3,float _n4,float _n5,float _n6) {
	int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);
    int32_t n3=toLong(_n3);
    int32_t n4=toLong(_n4);
    int32_t n5=toLong(_n5);
    int32_t n6=toLong(_n6);
    
    uint8_t len = 24;
    uint8_t msg_id = PID_RPY2;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2)+sum4byte(n3)+sum4byte(n4)+sum4byte(n5)+sum4byte(n6);
	
	sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1); 
    send4byte(s,n2);
    send4byte(s,n3);
    send4byte(s,n4); 
    send4byte(s,n5); 
    send4byte(s,n6); 
    send2byte(s,chk); //chk
}
static void send_pid_alt_param(int s,float _n1,float _n2,float _n3,float _n4,float _n5) {
	int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);
    int32_t n3=toLong(_n3);
    int32_t n4=toLong(_n4);
    int32_t n5=toLong(_n5);
    
    uint8_t len = 20;
    uint8_t msg_id = PID_ALT;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2)+sum4byte(n3)+sum4byte(n4)+sum4byte(n5);
	
	sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1); 
    send4byte(s,n2);
    send4byte(s,n3);
    send4byte(s,n4); 
    send4byte(s,n5); 
    send2byte(s,chk); //chk
}
static void send_pid_nav_param(int s,float _n1,float _n2,float _n3,float _n4,float _n5) {
	int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);
    int32_t n3=toLong(_n3);
    int32_t n4=toLong(_n4);
    int32_t n5=toLong(_n5);
    
    uint8_t len = 20;
    uint8_t msg_id = PID_NAV;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2)+sum4byte(n3)+sum4byte(n4)+sum4byte(n5);
	
	sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1); 
    send4byte(s,n2);
    send4byte(s,n3);
    send4byte(s,n4); 
    send4byte(s,n5); 
    send2byte(s,chk); //chk
}
static void send_spd_param(int s,float _n1,float _n2) {
	int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);

    uint8_t len = 8;
    uint8_t msg_id = PID_SPD;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2);
	
	sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1); 
    send4byte(s,n2);
    send2byte(s,chk); //chk
}
static void send_nav_iner_param(int s,float _n1,float _n2,float _n3,float _n4,float _n5) {
	int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);
    int32_t n3=toLong(_n3);
    int32_t n4=toLong(_n4);
    int32_t n5=toLong(_n5);
    
    uint8_t len = 20;
    uint8_t msg_id = NAV_INER;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2)+sum4byte(n3)+sum4byte(n4)+sum4byte(n5);
	
	sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1); 
    send4byte(s,n2);
    send4byte(s,n3);
    send4byte(s,n4); 
    send4byte(s,n5); 
    send2byte(s,chk); //chk
}
static void send_mode_param(int s,float _n1,float _n2,float _n3,float _n4,float _n5,float _n6) {
	int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);
    int32_t n3=toLong(_n3);
    int32_t n4=toLong(_n4);
    int32_t n5=toLong(_n5);
    int32_t n6=toLong(_n6);
    
    uint8_t len = 24;
    uint8_t msg_id = MODE_CFG;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2)+sum4byte(n3)+sum4byte(n4)+sum4byte(n5)+sum4byte(n6);
	
	sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1); 
    send4byte(s,n2);
    send4byte(s,n3);
    send4byte(s,n4); 
    send4byte(s,n5); 
    send4byte(s,n6); 
    send2byte(s,chk); //chk
}
static void send_vision_data(int s,float _n1,float _n2,float _n3,uint16_t n4) {
	int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);
	int32_t n3=toLong(_n3);
	
	uint8_t len = 14;
    uint8_t msg_id = VISION_DATA;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2)+sum4byte(n3)+sum2byte(n4);
    
    sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1);
    send4byte(s,n2); 
    send4byte(s,n3); 
    send2byte(s,n4);
    send2byte(s,chk); //chk
}
static void send_msf_data(int s,float _n1,float _n2,float _n3,float _n4,float _n5,float _n6) {
	int32_t n1=toLong(_n1);
    int32_t n2=toLong(_n2);
    int32_t n3=toLong(_n3);
    int32_t n4=toLong(_n4);
    int32_t n5=toLong(_n5);
    int32_t n6=toLong(_n6);
    
    uint8_t len = 24;
    uint8_t msg_id = MSF_DATA;
    uint16_t chk = HEADER + len + msg_id +sum4byte(n1)+sum4byte(n2)+sum4byte(n3)+sum4byte(n4)+sum4byte(n5)+sum4byte(n6);
	
	sendbyte(s,HEADER); //hdr
    sendbyte(s,len);    //len
    sendbyte(s,msg_id); //msg id
    send4byte(s,n1); 
    send4byte(s,n2);
    send4byte(s,n3);
    send4byte(s,n4); 
    send4byte(s,n5); 
    send4byte(s,n6); 
    send2byte(s,chk); //chk
}