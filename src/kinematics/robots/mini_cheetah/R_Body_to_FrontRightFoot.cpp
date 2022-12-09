/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:03:50 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_Body_to_FrontRightFoot.h"

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
  double t843;
  double t859;
  double t835;
  double t869;
  double t862;
  double t877;
  t843 = Cos(var1[2]);
  t859 = Sin(var1[1]);
  t835 = Cos(var1[1]);
  t869 = Sin(var1[0]);
  t862 = Sin(var1[2]);
  t877 = Cos(var1[0]);

  p_output1(0)=0. + t835*t843 - 1.*t859*t862;
  p_output1(1)=0. - 1.*t843*t859*t869 - 1.*t835*t862*t869;
  p_output1(2)=0. + t843*t859*t877 + t835*t862*t877;
  p_output1(3)=0.;
  p_output1(4)=0. + t877;
  p_output1(5)=0. + t869;
  p_output1(6)=0. - 1.*t843*t859 - 1.*t835*t862;
  p_output1(7)=0. - 1.*t835*t843*t869 + t859*t862*t869;
  p_output1(8)=0. + t835*t843*t877 - 1.*t859*t862*t877;
}


       
Eigen::Matrix<double,3,3> R_Body_to_FrontRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void R_Body_to_FrontRightFoot(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



