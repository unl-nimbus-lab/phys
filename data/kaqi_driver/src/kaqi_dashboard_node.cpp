#include "kaqi_dashboard.h"
#include <signal.h>
#include <QPlastiqueStyle>

void quit(int sig)
{
    exit(0);
}

int main(int argc, char ** argv)
{
    ros::init( argc, argv, "kaqi_dashboard_node");

    QApplication app( argc, argv );
    QApplication::setStyle("plastique");
//    QApplication::setStyle("cleanlooks");

    KaqiDashboard* dashboard = new KaqiDashboard();
    dashboard->setWindowFlags( Qt::WindowStaysOnTopHint );

    signal(SIGINT, quit);

    dashboard->show();
    app.exec();

    delete dashboard;

    ros::shutdown();
    return 0;
}
