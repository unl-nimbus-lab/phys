#define instability_fix 1   //for inv_sqrt







// #define applyDeadband(value, deadband)  \
//   if(abs(value) < deadband) {           \
//     value = 0;                          \
//   } else if(value > 0){                 \
//     value -= deadband;                  \
//   } else if(value < 0){                 \
//     value += deadband;                  \
//   }
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define wrap_pi(x) (x < -3.14 ? x+6.28 : (x > 3.14 ? x - 6.28: x))
#define HALF_M_PI 1.570796
#define radians(x) (x*3.14159265359/180)
#define DEG_TO_RAD 0.017453293










//--------------------Auxilary function of mathematic with optimization for light-weight programing--------------
static float _atan2(float y, float x){
  float z = y / x;
  int16_t a;
  if ( abs(y) < abs(x) ){
     a = 573 * z / (1.0f + 0.28f * z * z);
   if (x<0) {
     if (y<0) a -= 1800;
     else a += 1800;
   }
  } else {
   a = 900 - 573 * z / (z * z + 0.28f);
   if (y<0) a -= 1800;
  }
  return a/573.0f;
}
static float _asin(float x) {
  
  return 0.43600*x*x*x - 0.00812*x*x + 0.91734*x + 0.00162;
}
static float _sin(float x) {
  wrap_pi(x);  //limit range -pi to pi
  return  x*x*x*x*x*5.631332E-03 - x*x*x*1.551655E-01 + x*9.876877E-01 ;
}
static float _cos(float x) {
  wrap_pi(x);  //limit range -pi to pi
  return 0.03715827*x*x*x*x /*+ 0.00034810*x*x*x*/ - 0.49644615*x*x /*- 0.00018852*x */+ 1.00;
}
float InvSqrt (float x){ 
  union{  
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}
float invSqrt(float x) {
        if (instability_fix == 0)
        {
             union {
               float f;
               int32_t i;
               } y;

             y.f = x;
             y.i = 0x5f375a86 - (y.i >> 1);
             y.f = y.f * ( 1.5f - ( x * 0.5f * y.f * y.f ) );
             return y.f;
        }
        else if (instability_fix == 1)
        {
                /* close-to-optimal  method with low cost from
				http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root */
                uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
                float tmp = *(float*)&i;
                return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
        }
//        else
//        {
//                /* optimal but expensive method: */
//                return 1.0f *invSqrt(x);
//        }
}


//exponential
#ifdef __cplusplus
#define cast_uint32_t static_cast<uint32_t>
#else
#define cast_uint32_t (uint32_t)
#endif

static inline float
fasterpow2 (float p)
{
  float clipp = (p < -126) ? -126.0f : p;
  union { uint32_t i; float f; } v = { cast_uint32_t ( (1 << 23) * (clipp + 126.94269504f) ) };
  return v.f;
}

static inline float
fasterexp (float p)
{
  return fasterpow2 (1.442695040f * p);
}


//------------------------------------------------------------------------------------------------------


static float pythagorus(float x,float y) 
{
  float l = x*x+y*y;
  return l*invSqrt(l);
}


static float pythagorous3(float x,float y,float z) {
  float l = x*x+y*y+z*z;
  return l*invSqrt(l);
}









//--------------------------------------------------------------------LOCATION----------------------------------------------------------
#define LOCATION_SCALING_FACTOR 0.01113195f
#define LATLON_TO_M  0.01113195f
#define LATLON_TO_CM 1.113195f


struct Location
{
     double lat;
     double lng;
     double alt;
};

// return distance in meters between two locations
static float get_distance(float lat, float lng , float hlat,float hlong)
{
    float dlat              = (float)(lat - hlat);
    float dlong             = (float)(lng - hlong);
    return pythagorus(dlat, dlong) * LOCATION_SCALING_FACTOR;

}



// constrain a value
static float constrain_float(float amt, float low, float high) 
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


//------------------------------------------------------------------VECTOR 2 STRUCTURE--------------------------------------------------------

template <typename T>
struct Vector2
{
    T x, y;

    // trivial ctor
    Vector2<T>() {
        x = y = 0;
    }

    // setting ctor
    Vector2<T>(const T x0, const T y0) : x(x0), y(y0) {
    }

    // function call operator
    void operator ()(const T x0, const T y0)
    {
        x= x0; y= y0;
    }

    // test for equality
    bool operator ==(const Vector2<T> &v) const;

    // test for inequality
    bool operator !=(const Vector2<T> &v) const;

    // negation
    Vector2<T> operator -(void) const;

    // addition
    Vector2<T> operator +(const Vector2<T> &v) const;

    // subtraction
    Vector2<T> operator -(const Vector2<T> &v) const;

    // uniform scaling
    Vector2<T> operator *(const T num) const;

    // uniform scaling
    Vector2<T> operator  /(const T num) const;

    // addition
    Vector2<T> &operator +=(const Vector2<T> &v);

    // subtraction
    Vector2<T> &operator -=(const Vector2<T> &v);

    // uniform scaling
    Vector2<T> &operator *=(const T num);

    // uniform scaling
    Vector2<T> &operator /=(const T num);

    // dot product
    T operator *(const Vector2<T> &v) const;

    // cross product
    T operator %(const Vector2<T> &v) const;

    // computes the angle between this vector and another vector
    float angle(const Vector2<T> &v2) const;

    // computes the angle in radians between the origin and this vector
    T angle(void) const;

    // check if any elements are NAN
    bool is_nan(void) const;

    // check if any elements are infinity
    bool is_inf(void) const;

    // zero the vector
    void zero()
    {
        x = y = 0;
    }

    // gets the length of this vector squared
    T   length_squared() const
    {
        return (T)(*this * *this);
    }

    // gets the length of this vector
    float           length(void) const;

    // normalizes this vector
    void    normalize()
    {
        *this/=length();
    }

    // returns the normalized vector
    Vector2<T>  normalized() const
    {
        return *this/length();
    }

    // reflects this vector about n
    void    reflect(const Vector2<T> &n)
    {
        Vector2<T>        orig(*this);
        project(n);
        *this= *this*2 - orig;
    }

    // projects this vector onto v
    void    project(const Vector2<T> &v)
    {
        *this= v * (*this * v)/(v*v);
    }

    // returns this vector projected onto v
    Vector2<T>  projected(const Vector2<T> &v)
    {
        return v * (*this * v)/(v*v);
    }
};

typedef Vector2<int16_t>        Vector2i;
typedef Vector2<uint16_t>       Vector2ui;
typedef Vector2<int32_t>        Vector2l;
typedef Vector2<uint32_t>       Vector2ul;
typedef Vector2<float>          Vector2f;

/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * vector3.cpp
 * Copyright (C) Andrew Tridgell 2012
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


template <typename T>
float Vector2<T>::length(void) const
{
	return pythagorus(x, y);
}


// dot product
template <typename T>
T Vector2<T>::operator *(const Vector2<T> &v) const
{
    return x*v.x + y*v.y;
}

// cross product
template <typename T>
T Vector2<T>::operator %(const Vector2<T> &v) const
{
    return x*v.y - y*v.x;
}

template <typename T>
Vector2<T> &Vector2<T>::operator *=(const T num)
{
    x*=num; y*=num;
    return *this;
}

template <typename T>
Vector2<T> &Vector2<T>::operator /=(const T num)
{
    x /= num; y /= num;
    return *this;
}

template <typename T>
Vector2<T> &Vector2<T>::operator -=(const Vector2<T> &v)
{
    x -= v.x; y -= v.y;
    return *this;
}

template <typename T>
bool Vector2<T>::is_nan(void) const
{
    return isnan(x) || isnan(y);
}

template <typename T>
bool Vector2<T>::is_inf(void) const
{
    return isinf(x) || isinf(y);
}

template <typename T>
Vector2<T> &Vector2<T>::operator +=(const Vector2<T> &v)
{
    x+=v.x; y+=v.y;
    return *this;
}

template <typename T>
Vector2<T> Vector2<T>::operator /(const T num) const
{
    return Vector2<T>(x/num, y/num);
}

template <typename T>
Vector2<T> Vector2<T>::operator *(const T num) const
{
    return Vector2<T>(x*num, y*num);
}

template <typename T>
Vector2<T> Vector2<T>::operator -(const Vector2<T> &v) const
{
    return Vector2<T>(x-v.x, y-v.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator +(const Vector2<T> &v) const
{
    return Vector2<T>(x+v.x, y+v.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator -(void) const
{
    return Vector2<T>(-x,-y);
}

template <typename T>
bool Vector2<T>::operator ==(const Vector2<T> &v) const
{
    return (x==v.x && y==v.y);
}

template <typename T>
bool Vector2<T>::operator !=(const Vector2<T> &v) const
{
    return (x!=v.x && y!=v.y);
}

template <typename T>
float Vector2<T>::angle(const Vector2<T> &v2) const
{
    float len = this->length() * v2.length();
    if (len <= 0) {
        return 0.0f;
    }
    float cosv = ((*this)*v2) / len;
    if (fabsf(cosv) >= 1) {
        return 0.0f;
    }
    return acosf(cosv);
}

// only define for float
template float Vector2<float>::length(void) const;
template float Vector2<float>::operator *(const Vector2<float> &v) const;
template float Vector2<float>::operator %(const Vector2<float> &v) const;
template Vector2<float> &Vector2<float>::operator *=(const float num);
template Vector2<float> &Vector2<float>::operator /=(const float num);
template Vector2<float> &Vector2<float>::operator -=(const Vector2<float> &v);
template Vector2<float> &Vector2<float>::operator +=(const Vector2<float> &v);
template Vector2<float> Vector2<float>::operator /(const float num) const;
template Vector2<float> Vector2<float>::operator *(const float num) const;
template Vector2<float> Vector2<float>::operator +(const Vector2<float> &v) const;
template Vector2<float> Vector2<float>::operator -(const Vector2<float> &v) const;
template Vector2<float> Vector2<float>::operator -(void) const;
template bool Vector2<float>::operator ==(const Vector2<float> &v) const;
template bool Vector2<float>::operator !=(const Vector2<float> &v) const;
template bool Vector2<float>::is_nan(void) const;
template bool Vector2<float>::is_inf(void) const;
template float Vector2<float>::angle(const Vector2<float> &v) const;




//------------------------------------------------------------------VECTOR 3 STRUCTURE------------------------------------------------------


#include <math.h>
#include <string.h>


//template <typename T>
//class Matrix3;

template <typename T>
class Vector3
{

public:
    T        x, y, z;

    // trivial ctor
    Vector3<T>() {
        x = y = z = 0;
    }

    // setting ctor
    Vector3<T>(const T x0, const T y0, const T z0) : x(x0), y(y0), z(z0) {
    }

    // function call operator
    void operator ()(const T x0, const T y0, const T z0)
    {
        x= x0; y= y0; z= z0;
    }

    // test for equality
    bool operator ==(const Vector3<T> &v) const;

    // test for inequality
    bool operator !=(const Vector3<T> &v) const;

    // negation
    Vector3<T> operator -(void) const;

    // addition
    Vector3<T> operator +(const Vector3<T> &v) const;

    // subtraction
    Vector3<T> operator -(const Vector3<T> &v) const;

    // uniform scaling
    Vector3<T> operator *(const T num) const;

    // uniform scaling
    Vector3<T> operator  /(const T num) const;

    // addition
    Vector3<T> &operator +=(const Vector3<T> &v);

    // subtraction
    Vector3<T> &operator -=(const Vector3<T> &v);

    // uniform scaling
    Vector3<T> &operator *=(const T num);

    // uniform scaling
    Vector3<T> &operator /=(const T num);

    // allow a vector3 to be used as an array, 0 indexed
    T & operator[](uint8_t i) {
        T *_v = &x;
        return _v[i];
    }

    const T & operator[](uint8_t i) const {
        const T *_v = &x;
        return _v[i];
    }

    // dot product
    T operator *(const Vector3<T> &v) const;

//    // multiply a row vector by a matrix, to give a row vector
//    Vector3<T> operator *(const Matrix3<T> &m) const;

//    // multiply a column vector by a row vector, returning a 3x3 matrix
//    Matrix3<T> mul_rowcol(const Vector3<T> &v) const;

    // cross product
    Vector3<T> operator %(const Vector3<T> &v) const;

    // computes the angle between this vector and another vector
    float angle(const Vector3<T> &v2) const;

    // check if any elements are NAN
    bool is_nan(void) const;

    // check if any elements are infinity
    bool is_inf(void) const;

    // check if all elements are zero
    bool is_zero(void) const { return x==0 && y == 0 && z == 0; }

//    // rotate by a standard rotation
//    void rotate(enum Rotation rotation);

    // gets the length of this vector squared
    T  length_squared() const
    {
        return (T)(*this * *this);
    }

    // gets the length of this vector
    float length(void) const;

    // normalizes this vector
    void normalize()
    {
        *this /= length();
    }

    // zero the vector
    void zero()
    {
        x = y = z = 0;
    }

    // returns the normalized version of this vector
    Vector3<T> normalized() const
    {
        return *this/length();
    }

    // reflects this vector about n
    void  reflect(const Vector3<T> &n)
    {
        Vector3<T>        orig(*this);
        project(n);
        *this = *this*2 - orig;
    }

    // projects this vector onto v
    void project(const Vector3<T> &v)
    {
        *this= v * (*this * v)/(v*v);
    }

    // returns this vector projected onto v
    Vector3<T> projected(const Vector3<T> &v) const
    {
        return v * (*this * v)/(v*v);
    }


};

typedef Vector3<int16_t>                Vector3i;
typedef Vector3<uint16_t>               Vector3ui;
typedef Vector3<int32_t>                Vector3l;
typedef Vector3<uint32_t>               Vector3ul;
typedef Vector3<float>                  Vector3f;



//// vector cross product
//template <typename T>
//Vector3<T> Vector3<T>::operator %(const Vector3<T> &v) const
//{
//    Vector3<T> temp(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
//    return temp;
//}
//
//// dot product
//template <typename T>
//T Vector3<T>::operator *(const Vector3<T> &v) const
//{
//    return x*v.x + y*v.y + z*v.z;
//}
//
//template <typename T>
//float Vector3<T>::length(void) const
//{
//    return pythagorous3(x, y, z);
//}

// only define for signed numbers
//template void Vector3<float>::rotate(enum Rotation);
template float Vector3<float>::length(void) const;
template Vector3<float> Vector3<float>::operator %(const Vector3<float> &v) const;
template float Vector3<float>::operator *(const Vector3<float> &v) const;
// vector cross product
template <typename T>
Vector3<T> Vector3<T>::operator %(const Vector3<T> &v) const
{
    Vector3<T> temp(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    return temp;
}

// dot product
template <typename T>
T Vector3<T>::operator *(const Vector3<T> &v) const
{
    return x*v.x + y*v.y + z*v.z;
}

template <typename T>
float Vector3<T>::length(void) const
{
    return pythagorous3(x, y, z);
}

template <typename T>
Vector3<T> &Vector3<T>::operator *=(const T num)
{
    x*=num; y*=num; z*=num;
    return *this;
}

template <typename T>
Vector3<T> &Vector3<T>::operator /=(const T num)
{
    x /= num; y /= num; z /= num;
    return *this;
}

template <typename T>
Vector3<T> &Vector3<T>::operator -=(const Vector3<T> &v)
{
    x -= v.x; y -= v.y; z -= v.z;
    return *this;
}

template <typename T>
bool Vector3<T>::is_nan(void) const
{
    return isnan(x) || isnan(y) || isnan(z);
}

template <typename T>
bool Vector3<T>::is_inf(void) const
{
    return isinf(x) || isinf(y) || isinf(z);
}

template <typename T>
Vector3<T> &Vector3<T>::operator +=(const Vector3<T> &v)
{
    x+=v.x; y+=v.y; z+=v.z;
    return *this;
}

template <typename T>
Vector3<T> Vector3<T>::operator /(const T num) const
{
    return Vector3<T>(x/num, y/num, z/num);
}

template <typename T>
Vector3<T> Vector3<T>::operator *(const T num) const
{
    return Vector3<T>(x*num, y*num, z*num);
}

template <typename T>
Vector3<T> Vector3<T>::operator -(const Vector3<T> &v) const
{
    return Vector3<T>(x-v.x, y-v.y, z-v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator +(const Vector3<T> &v) const
{
    return Vector3<T>(x+v.x, y+v.y, z+v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator -(void) const
{
    return Vector3<T>(-x,-y,-z);
}

template <typename T>
bool Vector3<T>::operator ==(const Vector3<T> &v) const
{
    return (x==v.x && y==v.y && z==v.z);
}

template <typename T>
bool Vector3<T>::operator !=(const Vector3<T> &v) const
{
    return (x!=v.x && y!=v.y && z!=v.z);
}

template <typename T>
float Vector3<T>::angle(const Vector3<T> &v2) const
{
    return acosf(((*this)*v2) / (this->length()*v2.length()));
}

//// multiplication of transpose by a vector
//template <typename T>
//Vector3<T> Vector3<T>::operator *(const Matrix3<T> &m) const
//{
//    return Vector3<T>(*this * m.colx(),
//                      *this * m.coly(),
//                      *this * m.colz());
//}

// multiply a column vector by a row vector, returning a 3x3 matrix
//template <typename T>
//Matrix3<T> Vector3<T>::mul_rowcol(const Vector3<T> &v2) const
//{
//    const Vector3<T> v1 = *this;
//    return Matrix3<T>(v1.x * v2.x, v1.x * v2.y, v1.x * v2.z,
//                      v1.y * v2.x, v1.y * v2.y, v1.y * v2.z,
//                      v1.z * v2.x, v1.z * v2.y, v1.z * v2.z);
//}

// only define for float
//template void Vector3<float>::rotate(enum Rotation);
//template float Vector3<float>::length(void) const;
//template Vector3<float> Vector3<float>::operator %(const Vector3<float> &v) const;
//template float Vector3<float>::operator *(const Vector3<float> &v) const;
//template Vector3<float> Vector3<float>::operator *(const Matrix3<float> &m) const;
//template Matrix3<float> Vector3<float>::mul_rowcol(const Vector3<float> &v) const;
template Vector3<float> &Vector3<float>::operator *=(const float num);
template Vector3<float> &Vector3<float>::operator /=(const float num);
template Vector3<float> &Vector3<float>::operator -=(const Vector3<float> &v);
template Vector3<float> &Vector3<float>::operator +=(const Vector3<float> &v);
template Vector3<float> Vector3<float>::operator /(const float num) const;
template Vector3<float> Vector3<float>::operator *(const float num) const;
template Vector3<float> Vector3<float>::operator +(const Vector3<float> &v) const;
template Vector3<float> Vector3<float>::operator -(const Vector3<float> &v) const;
template Vector3<float> Vector3<float>::operator -(void) const;
template bool Vector3<float>::operator ==(const Vector3<float> &v) const;
template bool Vector3<float>::operator !=(const Vector3<float> &v) const;
template bool Vector3<float>::is_nan(void) const;
template bool Vector3<float>::is_inf(void) const;
template float Vector3<float>::angle(const Vector3<float> &v) const;
////-------------------------------------------------------------------------------------------------------------------------------
static void norm_z_ef ( Vector3f  *temp)
{
  float norm = invSqrt(temp->x*temp->x + temp->y*temp->y + temp->z*temp->z);
//  temp->normalized();
  temp->x *=norm;
  temp->y *=norm;
  temp->z *=norm;
}
//
//

