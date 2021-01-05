/**
 * @author Carsten Könemann
 */
#ifndef __FPS_CALCULATOR__
#define __FPS_CALCULATOR__

#include <queue>

static const unsigned int MAX_SAMPLES = 5;

class FPSCalculator
{
  public:
    // Default constructor
    FPSCalculator();
    // Default destructor
    ~FPSCalculator();

    // Return average number of fps over MAX_SAMPLES
    float get_fps();

    // Update FPS counter.
    // Call inside your main loop.
    void update();
  
  protected:
    // The number of frames already drawn this second
    unsigned int frame_count;
    // System time at previous FPS calculation
    double t_previous,
    // Total number of FPS during the last MAX_SAMPLES
           fps_total;
    // All individual numbers of FPS of the last MAX_SAMPLES
    std::queue<int> fps_queue;
};

#endif //__FPS_CALCULATOR__
