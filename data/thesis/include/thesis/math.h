/**
 * @author Carsten Könemann
 */
#ifndef __MATH__
#define __MATH__

#include <math.h>
#include <limits.h>

// Angular dimensions
static const double RAD = 180.0 / M_PI;
static const double DEG = M_PI  / 180.0;

// Linear interpolation, normalized ratio, one step
static const double LERP_STEP = 1.0 / INT_MAX;

/**
 * @return The result of the linear interpolation between a and b.
 *         r = 0.0 will return a,
 *         r = 1.0 will return b.
 */
inline double lerp(double a, double b, double r)
{
  return (a * (1.0 - r) + b * r);
}

/**
 * @return The average of a (weighted by r_a) and b (weighted by r_b).
 */
inline double average(double a, double b, unsigned int r_a, unsigned int r_b)
{
  return (a * r_a + b * r_b) / (r_a + r_b);
}

#endif //__MATH__
