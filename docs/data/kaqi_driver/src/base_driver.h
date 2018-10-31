#ifndef BASE_DRIVER_H
#define BASE_DRIVER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/timer.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <kaqi_msgs/BatteryState.h>
#include <kaqi_msgs/ControlBreaker.h>
#include <kaqi_msgs/BaseParam.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <sound_play/sound_play.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <tf/transform_broadcaster.h>
#include <serial/serial.h>
#include <cstdio>
#include <stdlib.h>
#include <thread>
#include <mutex>
#include <sstream>

/*
motor 360 degree 13 pulses from encoder
1/60, wheel rotates 360 degree 780 pulses
motor no load RPM 183
183 / 60.0 * 780 * 1.0/50.0 = 47.58
0.02s <-> 47.58
1s <-> 2378
*/

#define ENCODER_OVERFLOW    65536
#define ENCODER_VALUE_MAX   20000

using namespace std;

namespace kaqi_driver
{

typedef union {
    float f;
    unsigned char bytes[4];
} bytes4Float;

struct EncoderData
{
    unsigned int seq;
    ros::Time stamp;
    int left;
    int right;
    bool is_initialized;

    EncoderData() : left(0), right(0), seq(0), is_initialized(false) { stamp = ros::Time::now(); }
    void setValue( int l, int r, ros::Time t ) { left = l; right = r; stamp = t; seq ++; }
    int &getLeft() { return left; }
    int &getRight() { return right; }
    ros::Time &getStamp() { return stamp; }
    unsigned int &getSeq() { return seq; }
};

struct GyroData
{
    const double adc_scale;
    unsigned int seq;
    ros::Time stamp;
    int adc_value;
    int offset;
    double scale;
    double value;
    bool is_initialized;

    GyroData() : adc_scale(1000.0 * M_PI / 180.0 / 32768.0 ), seq(0),
        is_initialized(false), adc_value(0),value(0),
        offset(0), scale(1.0), stamp(ros::Time::now()) {}
    void setAdcValue( int adc, ros::Time t)
    {
        adc_value = adc;
        stamp = t;
        value = (adc_value - offset) * scale * adc_scale;
        seq ++;
    }
    void setValue( double v, ros::Time t)
    {
        value = v;
        stamp = t;
        seq ++;
    }
    double &getValue() { return value; }
    ros::Time &getStamp() { return stamp; }
    unsigned int &getSeq() { return seq; }
};


struct ImuData
{
    // Param
    const double accel_adc_scale;
    const double gyro_adc_scale;
    int gyro_offset[3];
    int accel_offset[3];
    double gyro_scale[3];
    double accel_scale[3];
    // Data
    unsigned int seq;
    ros::Time stamp;
    int gyro_adc[3];
    int accel_adc[3];
    double gyro[3];
    double accel[3];
    bool is_initialized;

    ImuData() : gyro_adc_scale(500.0 * M_PI / 180.0 / 32768.0 ),
        accel_adc_scale(4.0*9.8/32768.0), seq(0),
        stamp(ros::Time::now()), is_initialized(false)
    {
        gyro_offset[0] = gyro_offset[1] = gyro_offset[2] = 0;
        accel_offset[0] = accel_offset[1] = accel_offset[2] = 0;
        gyro_scale[0] = gyro_scale[1] = gyro_scale[2] = 1.0;
        accel_scale[0] = accel_scale[1] = accel_scale[2] = 1.0;
    }

    void setAdcValue( std::string &buffer, int idx, ros::Time t )
    {
        std::vector<int> adcs;
        for( int i = 0; i < 6; i++, idx+= 2)
        {
            adcs.push_back((signed short)(((uint8_t)buffer[idx] << 8) + (uint8_t)buffer[idx+1]));
        }
        setAdcValue( adcs, t );
    }

    void setAdcValue( std::vector<int>& adcs, ros::Time t )
    {
        if( adcs.size() < 6 )
            return;

        gyro_adc[0] = adcs[0];
        gyro_adc[1] = adcs[1];
        gyro_adc[2] = adcs[2];
        accel_adc[0] = adcs[3];
        accel_adc[1] = adcs[4];
        accel_adc[2] = adcs[5];
        stamp = t;
        for( int i = 0; i < 3; i++)
        {
            gyro[i] = (gyro_adc[i] - gyro_offset[i]) * gyro_scale[i] * gyro_adc_scale;
            accel[i] = (accel_adc[i] - accel_offset[i]) * accel_scale[i] * accel_adc_scale;
        }
        seq ++;
    }
    ros::Time &getStamp() { return stamp; }
    unsigned int &getSeq() { return seq; }
};

enum { BreakerOff = 0, BreakerOn = 1, BreakerStandby = 2 };
struct BreakerStatus{
    bool valid;
    int index;
    int status;
    BreakerStatus() : valid(false), index(0), status(BreakerOff) {}
    BreakerStatus( bool vd, int idx, int st ) : valid(vd), index(idx), status(st) {}
};


class BaseDriver
{
public:
    BaseDriver();
    void reset();
    bool openPort();
    void closePort() { my_serial_->close(); }
    void uartWrite( std::vector<uint8_t> &data );

protected:
    void uartThread();
    void velocityCallback( const geometry_msgs::Twist::ConstPtr &msg );
    void controlBreakerCallback( const kaqi_msgs::ControlBreaker::ConstPtr &msg );
    void baseParamCallback( const kaqi_msgs::BaseParam::ConstPtr &msg );
    bool resetOdomCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );
    //
    void controllerTimerCallback( const ros::TimerEvent& event );
    void odomTimerCallback( const ros::TimerEvent& event );

private:
    void publishPortDiagnostic( const int &level,
                                const std::string &message);
    void publishOdomDiagnostic( const int &level,
                                const std::string &message);
    void publishImuDiagnostic( const int &level,
                               const std::string &message);
    void publishNodeDiagnostic( const int &level,
                                const std::string &message);
    void publishBatteryDiagnostic( const int &level,
                                   const std::string &message);
    void publishBreakerDiagnostic( const BreakerStatus &breaker,
                                   const std::string &description );
    //
    void sendBaseParam();
    void sendBreakerControl();
    void sendVelocity();
    //
    void calculateOdom();
    void publishOdom();
    void publishImuRaw();
    void publishImu();
    void publishJointStates();
    void publishBatteryState( double voltage );
    void checkTimeout();
    bool checkImuReading(short int gz);
    int encoderDelta(int last_reading, int reading );

    //
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::CallbackQueue my_callback_queue_;
    ros::AsyncSpinner* async_spinner_;
    sound_play::SoundClient* sound_client_;
    ros::Timer controller_timer_;
    ros::Timer odom_timer_;
    //
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber control_breaker_sub_;
    ros::Publisher odometry_pub_;
    ros::Publisher imu_raw_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher joint_states_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher diagnostic_pub_;
    ros::Subscriber base_param_sub_;
    ros::ServiceServer reset_odom_ss_;

    // Velocity
    std::mutex velocity_mutex_;
    geometry_msgs::PointStamped base_velocity_;

    // Encoders
    EncoderData encoder_value_;
    // Gyro
    ImuData imu_value_;
    GyroData gyro_value_;
    sensor_msgs::Imu imu_data_raw_;
    sensor_msgs::Imu imu_data_;

    // Base Battery Voltage
    int battery_adc_value_;
    double battery_voltage_;

    // Odometry
    geometry_msgs::Point odom_pose_;
    double left_wheel_angle_;
    double right_wheel_angle_;
    double caster_wheel_angle_;
    double caster_rotate_angle_;
    //
    std::mutex odometry_mutex_;
    nav_msgs::Odometry odometry_;

    //
    serial::Serial* my_serial_;
    thread* uart_thread_;

    // Parameters
    std::string port_;
    unsigned long baudrate_;
    unsigned long timeout_;

    // Flags
    bool publish_odom_tf_;
    bool publish_wheel_tf_;
    bool publish_wheel_joint_state_;
    //
    double controller_frequency_;
    double controller_timeout_;
    double odometry_frequency_;
    double odometry_timeout_;
    std::string left_wheel_joint_;
    std::string right_wheel_joint_;
    std::string caster_rotate_joint_;
    std::string caster_wheel_joint_;
    //
    std::string base_frame_;
    std::string odom_frame_;
    double wheel_radius_;
    double wheel_seperation_;
    int encoder_pulses_;
    double caster_wheel_diameter_;

    // Battery
    double battery_voltage_scale_;
    double battery_voltage_low_;
    double battery_voltage_stale_;

    // Base param
    kaqi_msgs::BaseParam base_param_buff_;
    // PID gains
    float pid_p_gain_buff_;
    float pid_i_gain_buff_;
    float pid_d_gain_buff_;
    float pid_clamp_gain_buff_;

    // breaker control
    BreakerStatus breaker_control_;
    std::vector<BreakerStatus> breaker_status_;
};

} // end of namespace

//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

#endif // BASE_DRIVER_H
