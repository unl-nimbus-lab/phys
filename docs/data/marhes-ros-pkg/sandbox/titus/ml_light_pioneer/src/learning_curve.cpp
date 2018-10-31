#include "ml_light_pioneer/learning_curve.h"

LearningCurve::LearningCurve()
{
  episode_ = 0;
  //gr_ = new mglGraphZB;
  //gr_->StartGIF("learning_curve.gif");
}

LearningCurve::~LearningCurve()
{
  //gr_->CloseGIF();
  //delete gr_;
}

void LearningCurve::UpdateSteps(int numberSteps)
{
  episode_++;
  steps_.push_back(numberSteps);
  
  //mglData y(episode_);
  //y.Set(steps_);
  //Plot(y);  
}

//void LearningCurve::Plot(mglData y)
//{
  //gr_->NewFrame();
  //gr_->Box();
  //gr_->Plot(y, "b2");
  //gr_->SetTicks('x', 1, -1);
  //gr_->Axis(mglPoint(0,0), mglPoint(y.nx, y.Maximal() * 1.25));
  //gr_->Axis();
  //gr_->Label('x',"Episode Number", 0);
  //gr_->Label('y',"Time Steps", 0);
  //gr_->Title("Learning Curve");
  //gr_->ShowImage("eog", true);
  //gr_->EndFrame();
//}

void LearningCurve::ShowImage()
{
  //mglData y(episode_);
  //y.Set(steps_);
  //gr_->Box();
  //gr_->Plot(y, "b2");
  //gr_->SetTicks('x', 1, -1);
  //gr_->Axis(mglPoint(0,0), mglPoint(y.nx, y.Maximal() * 1.25));
  //gr_->Axis();
  //gr_->Label('x',"Episode Number", 0);
  //gr_->Label('y',"Time Steps", 0);
  //gr_->Title("Learning Curve");
  //gr_->WritePNG("learningCurve.png");
}
/*
int main(int argc, char **argv)
{
  LearningCurve *lc = new LearningCurve();
  lc->UpdateSteps(15);
  lc->UpdateSteps(20);
  lc->UpdateSteps(20);
  lc->UpdateSteps(30);
  lc->UpdateSteps(20);
  
  delete lc;
  return 0;
}
*/
