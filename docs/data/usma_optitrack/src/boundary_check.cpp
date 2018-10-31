#include "mocap_optitrack/Quad.h"
#include "mocap_optitrack/QuadScripts.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "multiquad_script_test");
  ros::NodeHandle node;
  ros::Rate rate(FRAMES_PER_SEC);

  // Create quads
  Quad quad1("quad01");
  Quad quad2("quad02");

  // Group quads together
  Formation form;
  form.add_quad(quad1);
  form.add_quad(quad2);

  // form.add_script<Takeoff>(new Takeoff(0.0));


  ROS_INFO("Boundary checking starting...");

  while(node.ok()) {
    form.run();
    rate.sleep();
  }

  return 0;
}
