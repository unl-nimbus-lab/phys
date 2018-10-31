#include "mocap_optitrack/Quad.h"
#include "mocap_optitrack/QuadScripts.h"

int main(int argc, char** argv) {

  // ROS initialization stuff
  ros::init(argc, argv, "quadscript_example");
  ros::NodeHandle node;

  // Set the rate to run everything at. FRAMES_PER_SEC is set in
  // constants_config.h
  ros::Rate rate(FRAMES_PER_SEC);

  // Create Quad objects with the namespace corresponding to the
  // actual quadcopter, or the type of fake quad for FakeQuads.
  FakeQuad fake(WAND_MOVABLE);
  Quad quad1("quad01");
  Quad quad2("quad02");

  // Add quads to a formation to give them access to each other and
  // make group movements easier.
  Formation form;
  form.add_quad(fake);
  form.add_quad(quad1);
  form.add_quad(quad2);

  // Tells the entire formation to takeoff to the specified height.
  // Does not do anything for any FakeQuads in the formation.
  form.add_script<Takeoff>(new Takeoff(0.5));

  // Sets quad1 to the specified position (first 3), with the specified
  // orientation quaternion (last 4)
  quad1.add_script(new SetPose(-1, -0.7, 0.5,   0, 0, 0, -1));

  // Sets quad1's most recently added script to need a wand check, meaning
  // it won't move on to the next script until the wand signal is given.
  quad1.back()->set_needsWandCheck(true);

  // Same for quad2
  quad2.add_script(new SetPose(-1, 0.7, 0.5,   0, 0, 0, -1));
  quad2.back()->set_needsWandCheck(true);

  // Sets quads to follow the 0th index quad in the formation (last arg of ctor),
  // with the offset (x, y, z) specified in first 3 args.
  // 0th index quad in the formation is a WAND_MOVABLE FakeQuad, so the quads
  // will follow the imaginary point set by the FakeQuad.
  quad1.add_script(new FollowOffset(0.6, 0, 0,   0));
  quad2.add_script(new FollowOffset(-0.6, 0, 0,  0));


  // View QuadScripts.h for more details and documentation of how to
  // use specific scripts


  ROS_INFO("Starting...");

  // Loops FRAMES_PER_SEC times per second, running the publisher in each active
  // QuadScript every time. Instead of form.run(), can also use quad#.run() for
  // individual quads.
  while(node.ok()) {
    form.run();
    rate.sleep();
  }

  return 0;
}
