#include "kaqi_dashboard.h"

KaqiDashboard::KaqiDashboard( QWidget *parent ) :
    QWidget( parent ),
    private_nh_("~"),
    buttons_width_( 36 ),
    buttons_height_( 36 )
{
    // Callback queue
    nh_.setCallbackQueue(&my_callback_queue_);

    setupUi(this);
    setDefault();

    //
    control_breaker_publisher_ = nh_.advertise<kaqi_msgs::ControlBreaker>("breaker_control", 4);
    base_battery_subscriber_ = nh_.subscribe( "battery_state", 4, &KaqiDashboard::baseBatteryCallback, this );
    laptop_battery_subscriber_ = nh_.subscribe( "laptop_charge", 4, &KaqiDashboard::laptopBatteryCallback, this );
    diagnostic_info_subscriber_ = nh_.subscribe( "diagnostics_agg", 10, &KaqiDashboard::diagnosticsCallback, this );
    //
    async_spinner_ =  new ros::AsyncSpinner( 4, &my_callback_queue_ );
    async_spinner_->start();
}

KaqiDashboard::~KaqiDashboard()
{

}

void KaqiDashboard::quit()
{
    ros::shutdown();
}

void KaqiDashboard::setDefault()
{
    setWidgetLogLevel( port_button_, QPalette::Button, "Missing" );
    setWidgetLogLevel( joy_button_, QPalette::Button, "Missing" );
    setWidgetLogLevel( sound_button_, QPalette::Button, "Missing" );
    //
    setWidgetLogLevel( odom_button_, QPalette::Button, "Missing" );
    setWidgetLogLevel( kinect_button_, QPalette::Button, "Missing" );
    setWidgetLogLevel( imu_button_, QPalette::Button, "Missing" );
    //
    setWidgetLogLevel( breaker_0_button_, QPalette::Button, "Missing" );
    setWidgetLogLevel( breaker_1_button_, QPalette::Button, "Missing" );
    //
    setBaseBatteryVoltage( 0 );
    setLaptopBatteryVoltage( 0 );
}

void KaqiDashboard::setWidgetColor(QWidget* widget, QPalette::ColorRole acr, const QColor &color)
{
    QPalette pal = widget->palette( );
    pal.setColor( acr, color );
    widget->setPalette( pal );
    widget->setAutoFillBackground( true );
}

void KaqiDashboard::setWidgetLogLevel( QWidget* widget, QPalette::ColorRole acr, const std::string &value)
{
    if( value.compare("Missing") == 0 )
    {
        setWidgetColor( widget, acr, QColor( 211, 211, 211) );
        widget->setEnabled( false );
    }
    else
    {
        widget->setEnabled( true );

        //
        if( value.compare("OK") == 0 || value.compare("On") == 0 )
        {
            setWidgetColor( widget, acr, QColor( 0, 255, 0) );
        }
        else if( value.compare("Warning") == 0 || value.compare("Standby") == 0 )
        {
            setWidgetColor( widget, acr, QColor( 255, 255, 0) );
        }
        else if( value.compare("Error") == 0 || value.compare("Off") == 0 )
        {
            setWidgetColor( widget, acr, QColor( 255, 0, 0) );
        }
        else if( value.compare("Stale") == 0 )
        {
            setWidgetColor( widget, acr, QColor( 211, 211, 211) );
//            setWidgetColor( widget, acr, QColor( 219, 219, 112) );
        }
    }

}

void KaqiDashboard::setWidgetTooltip(QWidget* widget, const std::string &message )
{
    widget->setToolTip( QString::fromUtf8("<span style=\"color:black;\">")
                        + QString::fromStdString( message) + QString::fromUtf8("</span>"));
}

void KaqiDashboard::setBaseBatteryVoltage( double voltage, int percentage)
{
    base_battery_label_->setText( QString::number( voltage, 'g', 4) + QString("V") );
    if( percentage == 0 )
        percentage = (voltage - 9) * 100 / 3.6 ;
    int bar_value = std::max(0, std::min(percentage, 100) );
    Q_EMIT baseBatteryPercentageChanged( bar_value );
    //
    int alpha = 180;
    QColor bar_color;
    QColor label_color(255, 255, 255);
    std::string tooltip;

    if( voltage < 2.0 )
    {
        label_color.setRgb( 211, 211, 211 );
        bar_color.setRgb( 211, 211, 211 );
        tooltip = "Missing";
    }
    else if( voltage > 11.46 ) // 50%
    {
        bar_color.setRgb( 0, 255, 0, alpha );
        tooltip = "Full";
    }
    else if( voltage > 11.04 ) // 20%
    {
        bar_color.setRgb( 255, 255, 0, alpha );
        tooltip = "Low";
    }
    else
    {
        bar_color.setRgb( 255, 0, 0, alpha );
        tooltip = "Stale";
    }

    setWidgetColor( base_battery_label_, QPalette::Background, label_color );
    setWidgetColor( base_battery_bar_, QPalette::Highlight, bar_color );
    setWidgetTooltip( base_battery_label_, tooltip );
    setWidgetTooltip( base_battery_bar_, tooltip );
}

void KaqiDashboard::setLaptopBatteryVoltage( double voltage , int percentage)
{
    laptop_battery_label_->setText( QString::number( voltage, 'g', 4) + QString("V") );
    int bar_value = std::max(0, std::min(percentage, 100) );
    Q_EMIT laptopBatteryPercentageChanged( bar_value );
    //
    int alpha = 180;
    QColor bar_color;
    QColor label_color(255, 255, 255);
    std::string tooltip;

    if( voltage == 0 )
    {
        label_color.setRgb( 211, 211, 211 );
        bar_color.setRgb( 211, 211, 211 );
        tooltip = "Missing";
    }
    else if( bar_value > 50 ) // 50%
    {
        bar_color.setRgb( 0, 255, 0, alpha );
        tooltip = "Full";
    }
    else if( bar_value > 20 ) // 20%
    {
        bar_color.setRgb( 255, 255, 0, alpha );
        tooltip = "Low";
    }
    else
    {
        bar_color.setRgb( 255, 0, 0, alpha );
        tooltip = "Stale";
    }

    setWidgetColor( laptop_battery_label_, QPalette::Background, label_color );
    setWidgetColor( laptop_battery_bar_, QPalette::Highlight, bar_color );
    setWidgetTooltip( laptop_battery_label_, tooltip );
    setWidgetTooltip( laptop_battery_bar_, tooltip );
}

void KaqiDashboard::setBreakerStatus( QWidget* widget, const diagnostic_msgs::DiagnosticStatus &diag )
{
    std::string level = "Missing";

    if( diag.message.compare("Missing") == 0)
    {
        level = diag.message;
    }
    else
    {
        for( int j = 0; j < diag.values.size(); j++)
        {
            const diagnostic_msgs::KeyValue &keyvalue = diag.values[j];
            if( keyvalue.key.compare("Status") == 0 )
                level = keyvalue.value; // "On", "Off", "Standby"
        }
    }

    setWidgetLogLevel( widget, QPalette::Button, level );
    setWidgetTooltip( widget, level );
}

void KaqiDashboard::sliderChanged( int value )
{
    double voltage = 11.0 + value * 0.01 * 1.6;
    setBaseBatteryVoltage( voltage, value );
    setLaptopBatteryVoltage( voltage, value );
    ROS_INFO("Set voltage = %f, value = %d", voltage, value );
}

void KaqiDashboard::controlBreaker( int id, int control )
{
    kaqi_msgs::ControlBreaker msg;
    msg.id = id;
    msg.control = control;
    control_breaker_publisher_.publish( msg );
}

void KaqiDashboard::onB0Enable()
{
    controlBreaker(0, kaqi_msgs::ControlBreaker::ENABLE );
//    ROS_INFO("B0 Enable");
}

void KaqiDashboard::onB0Disable()
{
    controlBreaker(0, kaqi_msgs::ControlBreaker::DISABLE );
//    ROS_INFO("B0 Disable");
}

void KaqiDashboard::onB1Enable()
{
    controlBreaker(1, kaqi_msgs::ControlBreaker::ENABLE );
//    ROS_INFO("B1 Enable");
}

void KaqiDashboard::onB1Disable()
{
    controlBreaker(1, kaqi_msgs::ControlBreaker::DISABLE );
//    ROS_INFO("B1 Disable");
}

void KaqiDashboard::baseBatteryCallback(const kaqi_msgs::BatteryState::ConstPtr &msg)
{
    setBaseBatteryVoltage( msg->voltage );
//    ROS_INFO("Base battery voltage = %f", msg->voltage );
}

void KaqiDashboard::laptopBatteryCallback(const smart_battery_msgs::SmartBatteryStatus::ConstPtr &msg)
{
    double voltage = msg->voltage;
    setLaptopBatteryVoltage( voltage , msg->percentage );
//    ROS_INFO("Laptop battery voltage = %f, percentage = %d", voltage, msg->percentage );
}

void KaqiDashboard::diagnosticsCallback( const diagnostic_msgs::DiagnosticArray::ConstPtr &msg )
{
//    ROS_INFO("Diagnostics message %d", msg->header.seq );

    for( int i = 0; i < msg->status.size(); i++ )
    {
        const diagnostic_msgs::DiagnosticStatus &status = msg->status[i];
        if( status.name.compare("/Devices") == 0 )
        {
            for( int j = 0; j < status.values.size(); j++)
            {
                const diagnostic_msgs::KeyValue &keyvalue = status.values[j];
                if( keyvalue.key.compare("Serial") == 0 )
                {
                    setWidgetLogLevel( port_button_, QPalette::Button, keyvalue.value );
                }
                else if( keyvalue.key.compare("Joystick") == 0 )
                {
                    setWidgetLogLevel( joy_button_, QPalette::Button, keyvalue.value );
                }
                else if( keyvalue.key.compare("Sound") == 0 )
                {
                    setWidgetLogLevel( sound_button_, QPalette::Button, keyvalue.value );
                }
            }
        }
        else if( status.name.compare("/Sensors") == 0 )
        {
            for( int j = 0; j < status.values.size(); j++)
            {
                const diagnostic_msgs::KeyValue &keyvalue = status.values[j];
                if( keyvalue.key.compare("Kinect") == 0 )
                {
                    setWidgetLogLevel( kinect_button_, QPalette::Button, keyvalue.value );
                }
                else if( keyvalue.key.compare("Imu") == 0 )
                {
                    setWidgetLogLevel( imu_button_, QPalette::Button, keyvalue.value );
                }
                else if( keyvalue.key.compare("Odom") == 0 )
                {
                    setWidgetLogLevel( odom_button_, QPalette::Button, keyvalue.value );
                }
            }
        }
        else if( status.name.compare("/Devices/Serial/ base_driver_node: Base Driver Serial Port") == 0 )
        {
            setWidgetTooltip( port_button_, status.message );
        }
        else if( status.name.compare("/Devices/Joystick/Joystick Driver Status") == 0 )
        {
            setWidgetTooltip( joy_button_, status.message );
        }
        else if( status.name.compare("/Devices/Sound/sound_play: Node State") == 0 )
        {
            setWidgetTooltip( sound_button_, status.message );
        }
        else if( status.name.compare("/Sensors/Odom/base_driver_node: Odometry State") == 0 )
        {
            setWidgetTooltip( odom_button_, status.message );
        }
        else if( status.name.compare("/Sensors/Imu/base_driver_node: Imu State") == 0 )
        {
            setWidgetTooltip( imu_button_, status.message );
        }
        else if( status.name.compare("/Sensors/Kinect/Depth Image topic status") == 0 )
        {
            setWidgetTooltip( kinect_button_, status.message );
        }
        else if( status.name.compare("/Power System/Base Battery") == 0 )
        {
            if( status.message.compare("Missing") == 0 )
            {
                setBaseBatteryVoltage( 0, 0 );
            }
        }
        else if( status.name.compare("/Power System/Laptop Battery") == 0 )
        {
            if( status.message.compare("Missing") == 0 )
            {
                setLaptopBatteryVoltage( 0, 0);
            }
        }
        else if( status.name.compare("/Breakers/Breaker: 0") == 0 )
        {
            setBreakerStatus( breaker_0_button_, status );
        }
        else if( status.name.compare("/Breakers/Breaker: 1") == 0 )
        {
            setBreakerStatus( breaker_1_button_, status );
        }
    }
}
