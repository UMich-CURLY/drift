/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:52 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_Body_to_HindLeftFoot.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE static inline        /* use standard inline */
#endif

/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
INLINE double Power(double x, double y) { return pow(x, y); }
INLINE double Sqrt(double x) { return sqrt(x); }

INLINE double Abs(double x) { return fabs(x); }

INLINE double Exp(double x) { return exp(x); }
INLINE double Log(double x) { return log(x); }

INLINE double Sin(double x) { return sin(x); }
INLINE double Cos(double x) { return cos(x); }
INLINE double Tan(double x) { return tan(x); }

INLINE double Csc(double x) { return 1.0/sin(x); }
INLINE double Sec(double x) { return 1.0/cos(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }


/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y,x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

/*
 * Sub functions
 */
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  double t5168;
  double t5175;
  double t5160;
  double t5191;
  double t5179;
  double t5203;
  t5168 = Cos(var1[11]);
  t5175 = Sin(var1[10]);
  t5160 = Cos(var1[10]);
  t5191 = Sin(var1[9]);
  t5179 = Sin(var1[11]);
  t5203 = Cos(var1[9]);

  p_output1(0)=t5160*t5168 - 1.*t5175*t5179;
  p_output1(1)=t5168*t5175*t5191 + t5160*t5179*t5191;
  p_output1(2)=-1.*t5168*t5175*t5203 - 1.*t5160*t5179*t5203;
  p_output1(3)=0;
  p_output1(4)=t5203;
  p_output1(5)=t5191;
  p_output1(6)=t5168*t5175 + t5160*t5179;
  p_output1(7)=-1.*t5160*t5168*t5191 + t5175*t5179*t5191;
  p_output1(8)=t5160*t5168*t5203 - 1.*t5175*t5179*t5203;
}


       
Eigen::Matrix<double,3,3> R_Body_to_HindLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void R_Body_to_HindLeftFoot(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



