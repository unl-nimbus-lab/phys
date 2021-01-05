#ifndef KAQI_DASHBOARD_H
#define KAQI_DASHBOARD_H


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <kaqi_msgs/BatteryState.h>
#include <kaqi_msgs/ControlBreaker.h>
#include <smart_battery_msgs/SmartBatteryStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <stdlib.h>
#include <thread>
#include <mutex>

#include <QApplication>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QProgressBar>
#include <QTimer>
#include <QSpacerItem>
#include <QDialog>
#include <QDialogButtonBox>
#include <QMenu>

using namespace std;

class KaqiDashboard : public QWidget
{
    Q_OBJECT
// gui related code
public:
    QWidget* dashboard_widget_;
    QSlider* voltage_slider_;

    QWidget* dashboardWidget( QWidget* parent = 0)
    {
        // Widget
        QHBoxLayout* h_layout = new QHBoxLayout;
        QHBoxLayout* sub_h_layout;
        QVBoxLayout* sub_v_layout;
        QGroupBox* group_box;
        //
        port_button_ = new QPushButton("Com");
        port_button_->setFixedWidth( buttons_width_ );
        port_button_->setFixedHeight( buttons_height_ );
        sound_button_ = new QPushButton("Snd");
        sound_button_->setFixedWidth( buttons_width_ );
        sound_button_->setFixedHeight( buttons_height_ );
        joy_button_ = new QPushButton("Joy");
        joy_button_->setFixedWidth( buttons_width_ );
        joy_button_->setFixedHeight( buttons_height_ );
        sub_h_layout = new QHBoxLayout;
        sub_h_layout->addWidget( port_button_ );
        sub_h_layout->addWidget( sound_button_ );
        sub_h_layout->addWidget( joy_button_ );
        sub_h_layout->setSpacing(2);
        group_box = new QGroupBox("Devices");
        group_box->setLayout( sub_h_layout );
        group_box->setContentsMargins( 0, 10, 0, 0);
        group_box->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Fixed );
        //
        h_layout->addWidget( group_box );
        //
        imu_button_ = new QPushButton("Imu");
        imu_button_->setFixedWidth( buttons_width_ );
        imu_button_->setFixedHeight( buttons_height_ );
        kinect_button_ = new QPushButton("Knt");
        kinect_button_->setFixedWidth( buttons_width_ );
        kinect_button_->setFixedHeight( buttons_height_ );
        odom_button_ = new QPushButton("Odm");
        odom_button_->setFixedWidth( buttons_width_ );
        odom_button_->setFixedHeight( buttons_height_ );
        sub_h_layout = new QHBoxLayout;
        sub_h_layout->addWidget( odom_button_ );
        sub_h_layout->addWidget( imu_button_ );
        sub_h_layout->addWidget( kinect_button_ );
        sub_h_layout->setSpacing(2);
        group_box = new QGroupBox("Sensors");
        group_box->setContentsMargins( 0, 10, 0, 0);
        group_box->setLayout( sub_h_layout );
        group_box->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Fixed );
        //
        h_layout->addWidget( group_box );
        //
        QMenu* button_menu;
        //
        breaker_0_button_ = new QPushButton("B0");
        breaker_0_button_->setFixedWidth( buttons_width_ );
        breaker_0_button_->setFixedHeight( buttons_height_ );
        button_menu = new QMenu;
        b0_enable_action_ = button_menu->addAction("Enable");
        b0_disable_action_ = button_menu->addAction("Disable");
        breaker_0_button_->setMenu( button_menu );
        //
        breaker_1_button_ = new QPushButton("B1");
        breaker_1_button_->setFixedWidth( buttons_width_ );
        breaker_1_button_->setFixedHeight( buttons_height_ );
        button_menu = new QMenu;
        b1_enable_action_ = button_menu->addAction("Enable");
        b1_disable_action_ = button_menu->addAction("Disable");
        breaker_1_button_->setMenu( button_menu );
        //
        sub_h_layout = new QHBoxLayout;
        sub_h_layout->addWidget( breaker_0_button_ );
        sub_h_layout->addWidget( breaker_1_button_ );
        sub_h_layout->setSpacing(2);
        group_box = new QGroupBox("Breakers");
        group_box->setContentsMargins( 0, 10, 0, 0);
        group_box->setLayout( sub_h_layout );
        group_box->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Fixed );
        //
        h_layout->addWidget( group_box );
        //
        QLabel* base_battery_name = new QLabel("Base");
        base_battery_name->setFixedWidth(50);
        QLabel* laptop_battery_name = new QLabel("Laptop");
        laptop_battery_name->setFixedWidth(50);
        base_battery_bar_ = new QProgressBar();
        base_battery_bar_->setFixedWidth( buttons_width_ * 2 );
        base_battery_bar_->setFixedHeight( buttons_height_ / 2 - 3 );
        base_battery_bar_->setRange(0, 100);
        laptop_battery_bar_ = new QProgressBar();
        laptop_battery_bar_->setFixedWidth( buttons_width_ * 2 );
        laptop_battery_bar_->setFixedHeight( buttons_height_ / 2 - 3 );
        laptop_battery_bar_->setRange(0, 100);
        base_battery_label_ = new QLabel(QString::fromUtf8("0V"));
        base_battery_label_->setFixedWidth( buttons_width_ + 12);
        base_battery_label_->setFixedHeight( buttons_height_ / 2 - 3 );
        laptop_battery_label_ = new QLabel(QString::fromUtf8("0V"));
        laptop_battery_label_->setFixedWidth( buttons_width_ + 12 );
        laptop_battery_label_->setFixedHeight( buttons_height_ / 2 - 3 );
        sub_v_layout = new QVBoxLayout;
        //
        sub_h_layout = new QHBoxLayout;
        sub_h_layout->addWidget( base_battery_name );
        sub_h_layout->addWidget( base_battery_bar_ );
        sub_h_layout->addWidget( base_battery_label_ );
        sub_v_layout->addLayout( sub_h_layout );
        //
        sub_h_layout = new QHBoxLayout;
        sub_h_layout->addWidget( laptop_battery_name );
        sub_h_layout->addWidget( laptop_battery_bar_ );
        sub_h_layout->addWidget( laptop_battery_label_ );
        sub_v_layout->addLayout( sub_h_layout );
        //
        group_box = new QGroupBox("Power");
        group_box->setContentsMargins( 0, 10, 0, 0);
        group_box->setLayout( sub_v_layout );
        group_box->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Fixed );
        //
        h_layout->addWidget( group_box );

        //
        QWidget* dashboard_widget = new QWidget(parent);
        h_layout->setContentsMargins( 0, 0, 0, 0);
        dashboard_widget->setLayout( h_layout );
        dashboard_widget->setContentsMargins(0, 0, 0, 0);
        dashboard_widget->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Fixed );
        return dashboard_widget;
    }

    void setupUi(QWidget *window)
    {

        // Main layout
        QVBoxLayout* v_layout = new QVBoxLayout;
        // Dashboard Widget
        dashboard_widget_ = dashboardWidget();
        // Slider
        voltage_slider_ = new QSlider(Qt::Horizontal);
        voltage_slider_->setRange(0, 100);
        //
        v_layout->addWidget( dashboard_widget_ );
//        v_layout->addWidget( voltage_slider_ );


        // Set layout
        v_layout->setContentsMargins(0, 0, 0, 0);
        window->setLayout( v_layout );
        window->setContentsMargins(0, 0, 0, 0);
        window->setSizePolicy( QSizePolicy::Fixed, QSizePolicy::Fixed );
        window->setWindowTitle(QApplication::translate("Dashboard", "Dashboard", 0, QApplication::UnicodeUTF8) );

//        connect( voltage_slider_, SIGNAL(valueChanged(int)), this, SLOT(sliderChanged(int)) );
        connect( window, SIGNAL(destroyed()), this, SLOT(quit()) );
        connect( this, SIGNAL(baseBatteryPercentageChanged(int)), base_battery_bar_, SLOT(setValue(int)) );
        connect( this, SIGNAL(laptopBatteryPercentageChanged(int)), laptop_battery_bar_, SLOT(setValue(int)) );
        connect( b0_enable_action_, SIGNAL(triggered()), this, SLOT(onB0Enable()) );
        connect( b0_disable_action_, SIGNAL(triggered()), this, SLOT(onB0Disable()) );
        connect( b1_enable_action_, SIGNAL(triggered()), this, SLOT(onB1Enable()) );
        connect( b1_disable_action_, SIGNAL(triggered()), this, SLOT(onB1Disable()) );
    } // setupUi


protected Q_SLOTS:
    void quit();
    void sliderChanged(int value);
    void onB0Enable();
    void onB0Disable();
    void onB1Enable();
    void onB1Disable();

public Q_SLOTS:


Q_SIGNALS:
    void baseBatteryPercentageChanged(int value);
    void laptopBatteryPercentageChanged(int value);

public:
    void setDefault();
    void setWidgetColor(QWidget* widget, QPalette::ColorRole acr, const QColor &color);
    void setWidgetLogLevel( QWidget* widget, QPalette::ColorRole acr, const std::string &value);
    void setWidgetTooltip(QWidget* widget, const std::string &message );
    void setBaseBatteryVoltage( double voltage, int percentage = 0 );
    void setLaptopBatteryVoltage(double voltage, int percentage = 0);
    void setBreakerStatus( QWidget* widget, const diagnostic_msgs::DiagnosticStatus &diag );
    void controlBreaker( int id, int control );

protected:
    void baseBatteryCallback(const kaqi_msgs::BatteryState::ConstPtr &msg);
    void laptopBatteryCallback(const smart_battery_msgs::SmartBatteryStatus::ConstPtr &msg);
    void diagnosticsCallback( const diagnostic_msgs::DiagnosticArray::ConstPtr &msg );

private:
    //
    QPushButton* port_button_;
    QPushButton* joy_button_;
    QPushButton* sound_button_;
    QPushButton* imu_button_;
    QPushButton* kinect_button_;
    QPushButton* odom_button_;
    QPushButton* breaker_0_button_;
    QPushButton* breaker_1_button_;
    QAction* b0_enable_action_;
    QAction* b0_disable_action_;
    QAction* b1_enable_action_;
    QAction* b1_disable_action_;
    QProgressBar* base_battery_bar_;
    QProgressBar* laptop_battery_bar_;
    QLabel* base_battery_label_;
    QLabel* laptop_battery_label_;
    //
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::CallbackQueue my_callback_queue_;
    ros::AsyncSpinner* async_spinner_;
    ros::Subscriber base_battery_subscriber_;
    ros::Subscriber laptop_battery_subscriber_;
    ros::Subscriber diagnostic_info_subscriber_;
    ros::Publisher control_breaker_publisher_;

    // Parameter
    int buttons_width_;
    int buttons_height_;

public:
    explicit KaqiDashboard( QWidget *parent = 0 );
    ~KaqiDashboard();

private:

};

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


#endif // KAQIDASHBOARD_H
