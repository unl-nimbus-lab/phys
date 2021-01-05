/**
 * @author Carsten Könemann
 */

#include <thesis/fps_calculator.h>

#include <ros/time.h>

FPSCalculator::FPSCalculator()
{
  ros::Time::init();
  frame_count = 0;
  fps_total   = 0;
  t_previous  = ros::Time::now().toSec();
}

FPSCalculator::~FPSCalculator()
{
  // Default destructor
}

float FPSCalculator::get_fps()
{
  return fps_total / fps_queue.size();
}

void FPSCalculator::update()
{
  // Increase frame counter every frame
  frame_count++;
  // Get current time
  double t_current = ros::Time::now().toSec();
  // Every second...
  if(t_current - t_previous > 1.0)
  {
    // ...update FPS
    fps_total += frame_count;
    fps_queue.push(frame_count);
    if(fps_queue.size() > MAX_SAMPLES)
    {
      fps_total -= fps_queue.front();
      fps_queue.pop();
    }
    // ...reset previous time
    t_previous = t_current;
    // ...and reset frame counter
    frame_count = 0;
  }
}
