#include "base_driver.h"

namespace kaqi_driver
{

BaseDriver::BaseDriver()
    : private_nh_("~")
    , tf_broadcaster_()
    , encoder_value_()
    , gyro_value_()
    , imu_value_()
    , pid_p_gain_buff_( -1 )
    , pid_i_gain_buff_( -1 )
    , pid_d_gain_buff_( -1 )
    , pid_clamp_gain_buff_( -1 )
    , breaker_control_( false, 0, BreakerOff )
    , my_serial_(NULL)
{
    // Callback queue
    nh_.setCallbackQueue(&my_callback_queue_);

    //
    breaker_status_.resize( 2 );
    breaker_status_[0] = BreakerStatus( false, 0, BreakerOff );
    breaker_status_[1] = BreakerStatus( false, 1, BreakerOff );

    // parameters
    int baudrate, timeout;
    private_nh_.param<string>("port", port_, "/dev/ttyUSB0");
    private_nh_.param<int>("baudrate", baudrate, 230400);
    private_nh_.param<int>("timeout", timeout, 1000);
    baudrate_ = (unsigned long)baudrate;
    timeout_ = (unsigned long)timeout;
    //
    private_nh_.param<bool>("publish_odom_tf", publish_odom_tf_, true);
    private_nh_.param<bool>("publish_wheel_tf", publish_wheel_tf_, true);
    private_nh_.param<bool>("publish_wheel_joint_state", publish_wheel_joint_state_, true);
    //
    private_nh_.param<double>("controller_frequency", controller_frequency_, 20);
    private_nh_.param<double>("controller_timeout", controller_timeout_, 0.5);
    private_nh_.param<double>("odometry_frequency", odometry_frequency_, 50);
    private_nh_.param<double>("odometry_timeout", odometry_timeout_, 0.5);
    private_nh_.param<string>("left_wheel_joint", left_wheel_joint_, "left_wheel_joint");
    private_nh_.param<string>("right_wheel_joint", right_wheel_joint_, "right_wheel_joint");
    private_nh_.param<string>("caster_rotate_joint", caster_rotate_joint_, "caster_rotate_joint");
    private_nh_.param<string>("caster_wheel_joint", caster_wheel_joint_, "caster_wheel_joint");
    //
    private_nh_.param<string>("base_frame", base_frame_, "/base_footprint");
    private_nh_.param<string>("odom_frame", odom_frame_, "/odom");
    private_nh_.param<double>("wheel_radius", wheel_radius_, 0.03);
    private_nh_.param<double>("wheel_seperation", wheel_seperation_, 0.32);
    private_nh_.param<int>("encoder_pulses", encoder_pulses_, 3120);    // 13*60*4
    private_nh_.param<double>("caster_wheel_diameter", caster_wheel_diameter_, 0.05);
    //
    private_nh_.param<double>("battery_voltage_scale", battery_voltage_scale_, 20.16);
    private_nh_.param<double>("battery_voltage_low", battery_voltage_low_, 11.46);
    private_nh_.param<double>("battery_voltage_stale", battery_voltage_stale_, 11.04);
    //
    private_nh_.param<int>("gyro_z_offset", gyro_value_.offset, 27);
    private_nh_.param<double>("gyro_z_scale", gyro_value_.scale, 1.0);

    reset();

    sound_client_ = new sound_play::SoundClient();
    diagnostic_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>( "diagnostics", 10 );

    // Info
    cout << GREEN << "Open serial port:" << endl;
    cout << "  Port: " << port_ << endl;
    cout << "  Baudrate: " << baudrate_ << endl;
    cout << "  Timeout: " << timeout_ << RESET << endl;
    // Open serial port
    while( ros::ok() && !openPort() )
    {
        sound_client_->say("Fail  to  open  port.");
        publishNodeDiagnostic( diagnostic_msgs::DiagnosticStatus::WARN, "Serial port not opened yet...");
        ros::Duration(2.0).sleep();
    }

    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>( "odom", 10 );
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>( "joint_states", 10 );
    battery_pub_ = nh_.advertise<kaqi_msgs::BatteryState>( "battery_state", 10 );
    bool pub_imu;
    private_nh_.param<bool>("pub_imu", pub_imu, true);
    if( pub_imu )
    {
//        imu_raw_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 10);
    }

    // Base param
    base_param_sub_ = nh_.subscribe( "base_param", 10, &BaseDriver::baseParamCallback, this );
    // Reset odom service server
    reset_odom_ss_ = nh_.advertiseService("reset_odom", &BaseDriver::resetOdomCallback, this );

    //
    controller_timer_ = nh_.createTimer( ros::Duration(1.0/controller_frequency_), &BaseDriver::controllerTimerCallback, this );
    odom_timer_ = nh_.createTimer( ros::Duration(1.0/odometry_frequency_), &BaseDriver::odomTimerCallback, this );

    velocity_sub_ = nh_.subscribe( "base_controller/command", 10, &BaseDriver::velocityCallback, this );
    control_breaker_sub_ = nh_.subscribe( "breaker_control", 10, &BaseDriver::controlBreakerCallback, this );

    async_spinner_ =  new ros::AsyncSpinner( 6, &my_callback_queue_ );
    async_spinner_->start();

    uart_thread_ = new thread( &BaseDriver::uartThread, this );

    sound_client_->say("Base  node  started.");
    ROS_INFO("Base driver started.");
}

void BaseDriver::reset()
{
    // Velocity
    base_velocity_.header.seq = 0;
    base_velocity_.header.stamp = ros::Time::now();
    base_velocity_.point.x = 0;
    base_velocity_.point.y = 0;
    base_velocity_.point.z = 0;
    // Odometry
    odometry_.header.stamp = ros::Time::now();
    odometry_.header.frame_id = odom_frame_;
    odometry_.child_frame_id = base_frame_;
    odometry_.pose.pose.position.x = 0;
    odometry_.pose.pose.position.y = 0;
    odometry_.pose.pose.position.z = 0;
    odometry_.pose.pose.orientation = tf::createQuaternionMsgFromYaw( 0 );
    odometry_.twist.twist.linear.x = 0;
    odometry_.twist.twist.linear.y = 0;
    odometry_.twist.twist.linear.z = 0;
    odometry_.twist.twist.angular.x = 0;
    odometry_.twist.twist.angular.y = 0;
    odometry_.twist.twist.angular.z = 0;
    odometry_.pose.covariance[0] = pow(0.005, 2);   // 5mm
    odometry_.pose.covariance[7] = pow(0.005, 2);   // 5mm
    odometry_.pose.covariance[14] = std::numeric_limits<double>::max();
    odometry_.pose.covariance[21] = std::numeric_limits<double>::max();
    odometry_.pose.covariance[28] = std::numeric_limits<double>::max();
    odometry_.pose.covariance[35] = pow(M_PI/180.0, 2);
    odometry_.twist.covariance[0] = pow(0.005, 2);    // 5mm/s
    odometry_.twist.covariance[7] = pow(0.005, 2);
    odometry_.twist.covariance[14] = std::numeric_limits<double>::max();
    odometry_.twist.covariance[21] = std::numeric_limits<double>::max();
    odometry_.twist.covariance[28] = std::numeric_limits<double>::max();
    odometry_.twist.covariance[35] = pow(M_PI/180.0, 2);

    //
    imu_data_.header.frame_id = "gyro_link";
    imu_data_.orientation.x = imu_data_.orientation.y = imu_data_.orientation.z = 0;
    imu_data_.orientation.w = 1.0;
    imu_data_.orientation_covariance[0] = 1e6;
    imu_data_.orientation_covariance[4] = 1e6;
    imu_data_.orientation_covariance[8] = 1e-6;
    imu_data_.angular_velocity_covariance[0] = 1e6;
    imu_data_.angular_velocity_covariance[4] = 1e6;
    imu_data_.angular_velocity_covariance[8] = 1e-6;
    imu_data_.linear_acceleration_covariance[0] = -1;
    //
    imu_data_raw_.header.frame_id = "imu_link";
    imu_data_raw_.orientation.x = imu_data_raw_.orientation.y = imu_data_raw_.orientation.z = 0;
    imu_data_raw_.orientation.w = 1.0;
    imu_data_raw_.orientation_covariance[0] = 1e-6;
    imu_data_raw_.orientation_covariance[4] = 1e-6;
    imu_data_raw_.orientation_covariance[8] = 1e-6;
    imu_data_raw_.angular_velocity_covariance[0] = 1e-6;
    imu_data_raw_.angular_velocity_covariance[4] = 1e-6;
    imu_data_raw_.angular_velocity_covariance[8] = 1e-6;
    imu_data_raw_.linear_acceleration_covariance[0] = 1e-6;
    imu_data_raw_.linear_acceleration_covariance[4] = 1e-6;
    imu_data_raw_.linear_acceleration_covariance[8] = 1e-6;

}

void BaseDriver::uartWrite(std::vector<uint8_t> &data )
{
    if( my_serial_ == NULL )
        return;

    my_serial_->write(data);
    my_serial_->write(data);
    my_serial_->write(data);
}

void BaseDriver::uartThread()
{
    while( ros::ok() )
    {
        // Read line test
        std::string buffer;
        my_serial_->readline(buffer);
//        cout << buffer;

        // Process frame
        const size_t size = buffer.size();
        int idx = 0;
        while( idx < size )
        {
            if( buffer[idx] != '#' )    // start symbol
                idx ++;
            else
            {
                // serial frame id
                if( idx + 2 >= size )
                    break;

                // Encoders reading
                if( buffer[idx+1] == 'E' && buffer[idx+2] == 'C' )
                {
                    if( idx + 10 >= size )
                        break;

                    // Check sum?

                    // Read encoders value
                    int left = ((uint8_t)buffer[idx+3] << 8) + (uint8_t)buffer[idx+4];
                    int right = ((uint8_t)buffer[idx+5] << 8) + (uint8_t)buffer[idx+6];
                    int gyro_adc = (signed short)(((uint8_t)buffer[idx+7] << 8) + (uint8_t)buffer[idx+8]);
                    ros::Time stamp = ros::Time::now();
                    encoder_value_.setValue( left, right, stamp );
                    encoder_value_.is_initialized = true;
                    gyro_value_.setAdcValue( gyro_adc, stamp );
                    gyro_value_.is_initialized = true;
//                    imu_value_.setAdcValue( buffer, 7, stamp );
//                    imu_value_.is_initialized = true;

//                    ROS_INFO_STREAM( " gyro adc = " << gyro_value_.adc_value << ", gz = " << gyro_value_.value );
                    if( encoder_value_.getLeft() < 65536 && encoder_value_.getRight() < 65536)
                    {
                        calculateOdom();
                        publishOdom();
                        if( imu_pub_ )
                            publishImu();
//                        if( imu_raw_pub_ )
//                            publishImuRaw();
                        publishJointStates();
                    }
                    publishOdomDiagnostic(diagnostic_msgs::DiagnosticStatus::OK, "OK");
                    if( imu_pub_ )
                    {
                        if( checkImuReading(gyro_value_.adc_value) )
                            publishImuDiagnostic(diagnostic_msgs::DiagnosticStatus::OK, "OK");
                        else
                            publishImuDiagnostic(diagnostic_msgs::DiagnosticStatus::ERROR, "Wrong gyroscope z-aixs measurement");
                    }
                    //
                    idx += 10;
                }
                else if( buffer[idx+1] == 'B' && buffer[idx+2] == 'S' )
                {
                    // Read battery voltage
                    battery_adc_value_ = ((uint8_t)buffer[idx+3] << 8) + (uint8_t)buffer[idx+4];
                    battery_voltage_ = battery_adc_value_ * battery_voltage_scale_ / 4096.0;
                    publishBatteryState( battery_voltage_ );
                    if( battery_voltage_ > battery_voltage_low_ )
                        publishBatteryDiagnostic( diagnostic_msgs::DiagnosticStatus::OK, "Full");
                    else if( battery_voltage_ > battery_voltage_stale_)
                        publishBatteryDiagnostic( diagnostic_msgs::DiagnosticStatus::WARN, "Low");
                    else
                        publishBatteryDiagnostic( diagnostic_msgs::DiagnosticStatus::STALE, "Stale");
                    //
                    idx += 8;
                }
                else if( buffer[idx+1] == 'B' && buffer[idx+2] == 'K' )
                {
                    if( buffer[idx+3] == 0 || buffer[idx+3] == 1 )
                        breaker_status_[0] = BreakerStatus( true, 0, buffer[idx+3] );
                    else
                        breaker_status_[0] = BreakerStatus( false, 0, BreakerOff );
                    if( buffer[idx+4] == 0 || buffer[idx+4] == 1 )
                        breaker_status_[1] = BreakerStatus( true, 1, buffer[idx+4] );
                    else
                        breaker_status_[1] = BreakerStatus( false, 1, BreakerOff );

                    //
                    publishBreakerDiagnostic( breaker_status_[0], "Kinect power breaker" );
                    publishBreakerDiagnostic( breaker_status_[1], "Not use." );

//                    cout << YELLOW << "Breakers: " << (int)(buffer[idx+3]) << ", " << (int)(buffer[idx+4]) << RESET << endl;

                    idx += 6;
                }
                else
                {
                    idx ++;
                }
            }
        }
//        usleep(100);
    }
}


bool BaseDriver::openPort()
{
    my_serial_ = NULL;

    try{
        my_serial_ = new serial::Serial( port_, baudrate_, serial::Timeout::simpleTimeout(timeout_) );
    }catch( serial::IOException &e ){
        ROS_WARN_STREAM( e.what() );
        my_serial_ = NULL;
    }catch( serial::PortNotOpenedException &e )
    {
        ROS_WARN_STREAM( e.what() );
        my_serial_ = NULL;
    }catch( std::invalid_argument &e )
    {
        ROS_WARN_STREAM( e.what() );
        my_serial_ = NULL;
    }

    bool success = true;
    if( my_serial_ != NULL )
    {
        cout << GREEN << "Successed to open port." << RESET << endl;
        publishPortDiagnostic( diagnostic_msgs::DiagnosticStatus::OK, "Running" );
        success = true;
    }
    else
    {
        cout << YELLOW << "Failed to open port. Will try agin..." << RESET << endl;
        publishPortDiagnostic( diagnostic_msgs::DiagnosticStatus::WARN, "Failed to open serial port" );
        success = false;
    }

    return success;
}

void BaseDriver::publishPortDiagnostic( const int &level,
                                        const std::string &message)
{
    //
    diagnostic_msgs::DiagnosticArray diags;
    diags.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "base_driver_node: Base Driver Serial Port";
    status.level = level;
    status.message = message;
    //
    diagnostic_msgs::KeyValue keyvalue;
    keyvalue.key = "Port";
    keyvalue.value = port_;
    status.values.push_back( keyvalue );
    keyvalue.key = "Baudrate";
    keyvalue.value = std::to_string( baudrate_ );
    status.values.push_back( keyvalue );
    keyvalue.key = "Timeout";
    keyvalue.value = std::to_string( timeout_ );
    status.values.push_back( keyvalue );
    //
    diags.status.push_back( status );

    //
    diagnostic_pub_.publish( diags );
}

void BaseDriver::publishOdomDiagnostic( const int &level,
                                        const std::string &message)
{
    static ros::Time last_time = ros::Time::now();
    //
    diagnostic_msgs::DiagnosticArray diags;
    diags.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "base_driver_node: Odometry State";
    status.level = level;
    status.message = message;
    //
    diagnostic_msgs::KeyValue keyvalue;
    keyvalue.key = "Delta Time";
    keyvalue.value = std::to_string( (encoder_value_.stamp - last_time ).toSec() );
    status.values.push_back( keyvalue );
    keyvalue.key = "Encoder Left";
    keyvalue.value = std::to_string( encoder_value_.left );
    status.values.push_back( keyvalue );
    keyvalue.key = "Encoder Right";
    keyvalue.value = std::to_string( encoder_value_.right );
    status.values.push_back( keyvalue );
    //
    diags.status.push_back( status );

    //
    diagnostic_pub_.publish( diags );

    if( message.compare("Timeout") != 0)
        last_time = encoder_value_.stamp;
}

void BaseDriver::publishImuDiagnostic( const int &level,
                                       const std::string &message)
{
    static ros::Time last_time = ros::Time::now();
    //
    diagnostic_msgs::DiagnosticArray diags;
    diags.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "base_driver_node: Imu State";
    status.level = level;
    status.message = message;
    //
    diagnostic_msgs::KeyValue keyvalue;
    keyvalue.key = "Adc Value";
    keyvalue.value = std::to_string( gyro_value_.adc_value );
    status.values.push_back( keyvalue );
    keyvalue.key = "Gyro Value";
    keyvalue.value = std::to_string( gyro_value_.value );
    status.values.push_back( keyvalue );
    keyvalue.key = "Delta Time";
    keyvalue.value = std::to_string( (gyro_value_.stamp - last_time ).toSec() );
    status.values.push_back( keyvalue );
    keyvalue.key = "Adc Scale";
    keyvalue.value = std::to_string( gyro_value_.adc_scale );
    status.values.push_back( keyvalue );
    keyvalue.key = "Offset";
    keyvalue.value = std::to_string( gyro_value_.offset );
    status.values.push_back( keyvalue );
    keyvalue.key = "Scale";
    keyvalue.value = std::to_string( gyro_value_.scale );
    status.values.push_back( keyvalue );
    //
    diags.status.push_back( status );

    //
    diagnostic_pub_.publish( diags );

    if( message.compare("Timeout") != 0)
        last_time = gyro_value_.stamp;
}

void BaseDriver::publishBatteryDiagnostic( const int &level,
                                        const std::string &message)
{
    //
    diagnostic_msgs::DiagnosticArray diags;
    diags.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "Base Battery";
    status.level = level;
    status.message = message;
    status.hardware_id = "Base Control Board";
    //
    diagnostic_msgs::KeyValue keyvalue;
    keyvalue.key = "Voltage";
    keyvalue.value = std::to_string( battery_voltage_ );
    status.values.push_back( keyvalue );
    keyvalue.key = "Voltage Low Value";
    keyvalue.value = std::to_string( battery_voltage_low_ );
    status.values.push_back( keyvalue );
    keyvalue.key = "Voltage Stale Value";
    keyvalue.value = std::to_string( battery_voltage_stale_ );
    status.values.push_back( keyvalue );
    keyvalue.key = "ADC Value";
    keyvalue.value = std::to_string( battery_adc_value_ );
    status.values.push_back( keyvalue );
    keyvalue.key = "ADC Max";
    keyvalue.value = std::to_string( 4096 );
    status.values.push_back( keyvalue );
    keyvalue.key = "Scale";
    keyvalue.value = std::to_string( battery_voltage_scale_ );
    status.values.push_back( keyvalue );
    //
    diags.status.push_back( status );

    //
    diagnostic_pub_.publish( diags );
}

void BaseDriver::publishBreakerDiagnostic( const BreakerStatus &breaker,
                                           const std::string &description )
{
    //
    diagnostic_msgs::DiagnosticArray diags;
    diags.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "Breaker: " + std::to_string(breaker.index);
    if( breaker.valid )
        status.level = diagnostic_msgs::DiagnosticStatus::OK;
    else
        status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = breaker.valid ? "OK" : "Missing";
    status.hardware_id = "Base Control Board";
    //
    diagnostic_msgs::KeyValue keyvalue;
    keyvalue.key = "Valid";
    keyvalue.value = breaker.valid ? "true" : "false";;
    status.values.push_back( keyvalue );
    keyvalue.key = "Index";
    keyvalue.value = std::to_string( breaker.index );
    status.values.push_back( keyvalue );
    keyvalue.key = "Status";
    keyvalue.value = breaker.status ? "On" : "Off";
    status.values.push_back( keyvalue );
    keyvalue.key = "Description";
    keyvalue.value = description;
    status.values.push_back( keyvalue );
    //
    diags.status.push_back( status );

    //
    diagnostic_pub_.publish( diags );
}

void BaseDriver::publishNodeDiagnostic( const int &level,
                                        const std::string &message)
{
    //
    diagnostic_msgs::DiagnosticArray diags;
    diags.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "base_driver_node: Node State";
    status.level = level;
    status.message = message;
    //
    diagnostic_msgs::KeyValue keyvalue;
    keyvalue.key = "Port State";
    keyvalue.value = (my_serial_ == NULL)?"Close":"Open";
    status.values.push_back( keyvalue );
    keyvalue.key = "Controller Frequency";
    keyvalue.value = std::to_string( controller_frequency_ );
    status.values.push_back( keyvalue );
    keyvalue.key = "Controller Timeout";
    keyvalue.value = std::to_string( controller_timeout_ );
    status.values.push_back( keyvalue );
    keyvalue.key = "Odometry Frequency";
    keyvalue.value = std::to_string( odometry_frequency_ );
    status.values.push_back( keyvalue );
    keyvalue.key = "Odometry Timeout";
    keyvalue.value = std::to_string( odometry_timeout_ );
    status.values.push_back( keyvalue );
    //
    diags.status.push_back( status );

    //
    diagnostic_pub_.publish( diags );
}

void BaseDriver::sendVelocity()
{
    // Get value
    velocity_mutex_.lock();
    double interval = (ros::Time::now() - base_velocity_.header.stamp).toSec();
    double linear = base_velocity_.point.x;
    double angular = base_velocity_.point.z;
    // Check velocity timeout
    if( interval > controller_timeout_ )
    {
        linear = 0;
        angular = 0;
    }
    velocity_mutex_.unlock();

    // Velocity to motor speed
    float left_speed = linear / wheel_radius_ - angular * wheel_seperation_ * 0.5 / wheel_radius_;
    float right_speed = linear / wheel_radius_ + angular * wheel_seperation_ * 0.5 / wheel_radius_;

    //
    if( left_speed < -20 )
        left_speed = -20;
    else if( left_speed > 20 )
        left_speed = 20;
    //
    if( right_speed < -20 )
        right_speed = -20;
    else if( right_speed > 20 )
        right_speed = 20;

//    cout << MAGENTA << " set pwm as (LR): " << left_speed << ", " << right_speed << RESET << endl;

    // send motor speed
    const int num = 13;
    std::vector<uint8_t> data(num);
    data[0] = '#';
    data[1] = 'M';
    data[2] = 'S';
    bytes4Float ls, rs;
    ls.f = left_speed;
    rs.f = right_speed;
    data[3] = ls.bytes[0];
    data[4] = ls.bytes[1];
    data[5] = ls.bytes[2];
    data[6] = ls.bytes[3];
    data[7] = rs.bytes[0];
    data[8] = rs.bytes[1];
    data[9] = rs.bytes[2];
    data[10] = rs.bytes[3];
    data[11] = 0;
    for(int i = 0; i < (num-2); i++)
    {
        data[num-2] += data[i];
    }
    data[num-1] = '\n';
    uartWrite( data );
}

void BaseDriver::sendBreakerControl()
{
    if( !breaker_control_.valid )
        return;

    // send motor speed
    const int num = 7;
    std::vector<uint8_t> data(num);
    data[0] = '#';
    data[1] = 'B';
    data[2] = 'K';
    data[3] = breaker_control_.index;
    data[4] = breaker_control_.status;
    data[5] = 0;
    for(int i = 0; i < (num-2); i++)
    {
        data[num-2] += data[i];
    }
    data[num-1] = '\n';
    uartWrite( data );

    breaker_control_.valid = false;

//    cout << MAGENTA << " Set breaker: " << breaker_control_.index << " = ";
//    if( breaker_control_.status == BreakerOff )
//    {
//        cout << "Off" << RESET << endl;
//    }
//    else if( breaker_control_.status == BreakerOn )
//    {
//        cout << "On" << RESET << endl;
//    }
//    else if( breaker_control_.status == BreakerStandby )
//    {
//        cout << "Standy" << RESET << endl;
//    }

}

// key: 'P', 'I', 'D', 'C'
void BaseDriver::sendBaseParam()
{
    if( base_param_buff_.id >= 0 && base_param_buff_.key.size() > 0 )
    {
        ROS_INFO( "Send base param, id = %d, %s = %f", base_param_buff_.id, base_param_buff_.key.c_str(), base_param_buff_.value );

        const int num = 9;
        std::vector<uint8_t> data(num);

        // send p_gain
        data[0] = '#';
        data[1] = 'P';
        data[2] = base_param_buff_.key[0];
        bytes4Float bf;
        bf.f = base_param_buff_.value;
        data[3] = bf.bytes[0];
        data[4] = bf.bytes[1];
        data[5] = bf.bytes[2];
        data[6] = bf.bytes[3];
        data[7] = 0;
        for(int i = 0; i < (num-2); i++)
        {
            data[num-2] += data[i];
        }
        data[num-1] = '\n';
        uartWrite( data );

        base_param_buff_.id = -1;
    }
}

void BaseDriver::calculateOdom()
{
    static const double pulse_to_rad = M_PI * 2.0 / encoder_pulses_;
    static EncoderData encoder_last;

    if( !encoder_value_.is_initialized )
        return;

    if( !encoder_last.is_initialized )
    {
        left_wheel_angle_ = 0;
        right_wheel_angle_ = 0;
        caster_wheel_angle_ = 0;
        caster_rotate_angle_ = 0;
        odom_pose_.x = 0;
        odom_pose_.y = 0;
        odom_pose_.z = 0;
        encoder_last = encoder_value_;
        encoder_last.is_initialized = true;
        return;
    }

    // Check timeout?
    double dt = (encoder_value_.stamp - encoder_last.stamp).toSec();

    if( dt <= 0 )
        return;

    // calculate delta
    int delta_left = encoderDelta( encoder_last.getLeft(), encoder_value_.getLeft() );
    int delta_right = encoderDelta( encoder_last.getRight(), encoder_value_.getRight() );
    // store last value
    encoder_last = encoder_value_;

//    cout << MAGENTA << "Encoder interval= " << dt*1000.0
//         << ", (LR): " << delta_left << ", " << delta_right << RESET << endl;

    // Compute odometry
    // Wheel Angles
    double theta_left = delta_left * pulse_to_rad;
    double theta_right = delta_right * pulse_to_rad;
    // Wheel velocity
    double vel_left = theta_left * wheel_radius_ / dt;
    double vel_right = theta_right * wheel_radius_ / dt;
    // Base linear velocity and angular velocity
    double linear_velocity = (vel_left + vel_right) / 2.0;
    double angular_velocity = (vel_right - vel_left) / wheel_seperation_;
    ///
    double dx = 0, dy = 0;
    double vx = 0, vy = 0, va = 0;
    if( angular_velocity != 0)
    {
        double vw = linear_velocity / angular_velocity;
        //
        double dtheta = angular_velocity * dt;
        dx = - vw * sin(odom_pose_.z) + vw * sin(odom_pose_.z + dtheta);
        dy = vw * cos(odom_pose_.z) - vw * cos(odom_pose_.z + dtheta);
        //
        odom_pose_.x += dx;
        odom_pose_.y += dy;
        odom_pose_.z += dtheta;
        //
        if( odom_pose_.z >= (M_PI*2.0) )
            odom_pose_.z -= (M_PI*2.0);
        else if( odom_pose_.z <= (-M_PI*2.0) )
            odom_pose_.z += (M_PI*2.0);
    }
    else
    {
        double dl = vel_left * dt;
        dx = dl * cos(odom_pose_.z);
        dy = dl * sin(odom_pose_.z);
        odom_pose_.x += dx;
        odom_pose_.y += dy;

    }
    // Set joint angle
    left_wheel_angle_ += theta_left;
    right_wheel_angle_ += theta_right;
    caster_wheel_angle_ += linear_velocity * dt / caster_wheel_diameter_;
    caster_rotate_angle_ = 0;


    // Set odometry
    odometry_mutex_.lock();
    odometry_.header.stamp = encoder_value_.getStamp();
    // Position
    odometry_.pose.pose.position.x = odom_pose_.x;
    odometry_.pose.pose.position.y = odom_pose_.y;
    odometry_.pose.pose.orientation = tf::createQuaternionMsgFromYaw( odom_pose_.z );
    // Velocity
    odometry_.twist.twist.linear.x = dx / dt;
    odometry_.twist.twist.linear.y = dy / dt;
    odometry_.twist.twist.angular.z = angular_velocity;
    odometry_mutex_.unlock();
}

void BaseDriver::publishOdom()
{
    odometry_mutex_.lock();
    // Set tf
    geometry_msgs::TransformStamped trans;
    trans.header.stamp = odometry_.header.stamp;
//    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = odom_frame_;
    trans.child_frame_id = base_frame_;
    trans.transform.translation.x = odometry_.pose.pose.position.x;
    trans.transform.translation.y = odometry_.pose.pose.position.y;
    trans.transform.translation.z = 0;
    trans.transform.rotation = odometry_.pose.pose.orientation;
    // Publish odometry
    odometry_pub_.publish( odometry_ );
    // Publish odom tf
    if( publish_odom_tf_ )
        tf_broadcaster_.sendTransform( trans );
    odometry_mutex_.unlock();
}

void BaseDriver::publishImuRaw()
{
    imu_data_raw_.header.stamp = imu_value_.getStamp();
    imu_data_raw_.angular_velocity.x = imu_value_.gyro[0];
    imu_data_raw_.angular_velocity.y = imu_value_.gyro[1];
    imu_data_raw_.angular_velocity.z = imu_value_.gyro[2];
    imu_data_raw_.linear_acceleration.x = imu_value_.accel[0];
    imu_data_raw_.linear_acceleration.y = imu_value_.accel[1];
    imu_data_raw_.linear_acceleration.z = imu_value_.accel[2];
    imu_raw_pub_.publish( imu_data_raw_ );
}

void BaseDriver::publishImu()
{   static ros::Time last_stamp = ros::Time::now();

    double dt = (gyro_value_.getStamp() - last_stamp).toSec();
    last_stamp = gyro_value_.getStamp();
    //
    imu_data_.header.stamp = gyro_value_.getStamp();
    imu_data_.angular_velocity.z = gyro_value_.getValue();
    double yaw = tf::getYaw( imu_data_.orientation );
    yaw += imu_data_.angular_velocity.z * dt;
    imu_data_.orientation = tf::createQuaternionMsgFromYaw( yaw );
    imu_pub_.publish( imu_data_ );
}

void BaseDriver::publishJointStates()
{
    if( publish_wheel_joint_state_ )
    {
        sensor_msgs::JointState joint_states;
        joint_states.header.stamp = encoder_value_.stamp;
        joint_states.name.push_back( left_wheel_joint_ );
        joint_states.position.push_back( left_wheel_angle_ );
        joint_states.name.push_back( right_wheel_joint_ );
        joint_states.position.push_back( right_wheel_angle_ );
        joint_states.name.push_back( caster_rotate_joint_ );
        joint_states.position.push_back( caster_rotate_angle_ );
        joint_states.name.push_back( caster_wheel_joint_ );
        joint_states.position.push_back( caster_wheel_angle_ );

        joint_states_pub_.publish( joint_states );
    }

    //
    diagnostic_msgs::DiagnosticArray diags;
    diags.header.stamp = ros::Time::now();
    diags.status.resize(4);
    //
    diags.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
    diags.status[0].name = "Joint ("+left_wheel_joint_+")";
    diags.status[0].message = "OK";
    diags.status[0].values.resize(1);
    diags.status[0].values[0].key = "Position";
    diags.status[0].values[0].value = std::to_string( left_wheel_angle_ );
    //
    diags.status[1].level = diagnostic_msgs::DiagnosticStatus::OK;
    diags.status[1].name = "Joint ("+right_wheel_joint_+")";
    diags.status[1].message = "OK";
    diags.status[1].values.resize(1);
    diags.status[1].values[0].key = "Position";
    diags.status[1].values[0].value = std::to_string( right_wheel_angle_ );
    //
    diags.status[2].level = diagnostic_msgs::DiagnosticStatus::OK;
    diags.status[2].name = "Joint ("+caster_rotate_joint_+")";
    diags.status[2].message = "OK";
    diags.status[2].values.resize(1);
    diags.status[2].values[0].key = "Position";
    diags.status[2].values[0].value = std::to_string( caster_rotate_angle_ );
    //
    diags.status[3].level = diagnostic_msgs::DiagnosticStatus::OK;
    diags.status[3].name = "Joint ("+caster_wheel_joint_+")";
    diags.status[3].message = "OK";
    diags.status[3].values.resize(1);
    diags.status[3].values[0].key = "Position";
    diags.status[3].values[0].value = std::to_string( caster_wheel_angle_ );

    // Publish diagnostic message
    diagnostic_pub_.publish( diags );
}

void BaseDriver::publishBatteryState( double voltage )
{
    kaqi_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();
    msg.voltage = voltage;
    //
    battery_pub_.publish( msg );
}

void BaseDriver::velocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
//    cout << "Set velocity x = " << msg->linear.x
//         << ", y = " << msg->linear.y
//         << ", yaw = " << msg->angular.z << endl;

    velocity_mutex_.lock();
    base_velocity_.header.seq ++;
    base_velocity_.header.stamp = ros::Time::now();
    base_velocity_.point.x = msg->linear.x;
    base_velocity_.point.y = msg->linear.y;
    base_velocity_.point.z = msg->angular.z;
    velocity_mutex_.unlock();
}

void BaseDriver::controlBreakerCallback( const kaqi_msgs::ControlBreaker::ConstPtr &msg )
{
//    ROS_INFO("Control breaker: %d, %s", msg->id, (msg->control ? "Enable" : "Disable") );
    breaker_control_.status = msg->control;
    breaker_control_.index = msg->id;
    breaker_control_.valid = true;
}

void BaseDriver::baseParamCallback(const kaqi_msgs::BaseParam::ConstPtr &msg)
{
    base_param_buff_ = *msg;
    ROS_INFO("Base param, id = %d, %s = %f", msg->id, msg->key.c_str(), msg->value );
}

bool BaseDriver::resetOdomCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // Set odometry
    odometry_mutex_.lock();
    odometry_.header.stamp = ros::Time::now();
    // Position
    odometry_.pose.pose.position.x = 0;
    odometry_.pose.pose.position.y = 0;
    odometry_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    // Velocity
    odometry_.twist.twist.linear.x = 0;
    odometry_.twist.twist.linear.y = 0;
    odometry_.twist.twist.angular.z = 0;
    odometry_mutex_.unlock();

    // Reset gyro
    imu_data_.angular_velocity.z = 0;
    imu_data_.orientation = tf::createQuaternionMsgFromYaw( 0 );
}

void BaseDriver::controllerTimerCallback(const ros::TimerEvent &event)
{
    static int count = 0;
    count ++;
    if( count >= (controller_frequency_ / 5) )
    {
        count = 0;
        if( my_serial_ == NULL || my_serial_->isOpen() )
            publishPortDiagnostic( diagnostic_msgs::DiagnosticStatus::OK, "Opened" );
        else
            publishPortDiagnostic( diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to open" );
        //
        publishNodeDiagnostic( diagnostic_msgs::DiagnosticStatus::OK, "Running");
    }

    if( !my_serial_->isOpen() )
        return;

//    ROS_INFO_STREAM("timer callback...");

    // send velocity command
    sendVelocity();
    // send break control
    sendBreakerControl();
    // send base param
    sendBaseParam();
}

void BaseDriver::odomTimerCallback(const ros::TimerEvent &event)
{
    checkTimeout();
}

void BaseDriver::checkTimeout()
{
    static double last_odom_interval = 0;
    double interval = (ros::Time::now() - odometry_.header.stamp).toSec();
    if( interval > odometry_timeout_ )
    {
        if( (interval - last_odom_interval) > 3.0 )
        {
            ROS_WARN("odometry timeout, %f", interval);
            last_odom_interval += 3.0;
            sound_client_->say("Odometry  timeout");
            publishOdomDiagnostic(diagnostic_msgs::DiagnosticStatus::WARN, "Timeout");
        }
    }
    else
    {
        last_odom_interval = 0;
    }
}

bool BaseDriver::checkImuReading(short int gz)
{
    static short int last_gyro_z = 0;
    static unsigned int count = 0;

    if( last_gyro_z == gz )
        count ++;
    else
        count = 0;

    last_gyro_z = gz;

    if( count >= 1000 )
    {
        count = 1000;
        return false;
    }
    return true;
}

int BaseDriver::encoderDelta( int last_reading, int reading)
{
    int delta = reading - last_reading;
    if( delta > ENCODER_VALUE_MAX )
        delta -= ENCODER_OVERFLOW;
    else if( delta < -ENCODER_VALUE_MAX )
        delta += ENCODER_OVERFLOW;

//    cout << BLUE << " last = " << last_reading
//         << ", cur = " << reading
//         << ", delta = " << delta << RESET << endl;

    return delta;
}


} // end of namespace

