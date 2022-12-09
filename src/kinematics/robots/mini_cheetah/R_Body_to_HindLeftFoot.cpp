/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:03:56 GMT-05:00
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
  double t961;
  double t977;
  double t953;
  double t987;
  double t980;
  double t995;
  t961 = Cos(var1[11]);
  t977 = Sin(var1[10]);
  t953 = Cos(var1[10]);
  t987 = Sin(var1[9]);
  t980 = Sin(var1[11]);
  t995 = Cos(var1[9]);

  p_output1(0)=0. + t953*t961 - 1.*t977*t980;
  p_output1(1)=0. - 1.*t961*t977*t987 - 1.*t953*t980*t987;
  p_output1(2)=0. + t961*t977*t995 + t953*t980*t995;
  p_output1(3)=0.;
  p_output1(4)=0. + t995;
  p_output1(5)=0. + t987;
  p_output1(6)=0. - 1.*t961*t977 - 1.*t953*t980;
  p_output1(7)=0. - 1.*t953*t961*t987 + t977*t980*t987;
  p_output1(8)=0. + t953*t961*t995 - 1.*t977*t980*t995;
}


       
Eigen::Matrix<double,3,3> R_Body_to_HindLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void R_Body_to_HindLeftFoot(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



