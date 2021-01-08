#ifndef LEARNING_CURVE_H
#define LEARNING_CURVE_H

#include "ros/ros.h"
#include <mgl/mgl_zb.h>
#include <vector>

class LearningCurve
{
public:
  LearningCurve();
  ~LearningCurve();
  void UpdateSteps(int numberSteps);
  void ShowImage();

private:
  int episode_;
  std::vector<int> steps_;
  mglGraph *gr_;
  
  void Plot(mglData y);
};

#endif
