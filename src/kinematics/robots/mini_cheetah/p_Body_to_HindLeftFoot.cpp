/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:03:57 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_Body_to_HindLeftFoot.h"

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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  double t981;
  double t1003;
  double t1005;
  double t1007;
  double t1010;
  double t1015;
  double t1032;
  double t1037;
  double t1008;
  double t1011;
  double t1012;
  double t1016;
  double t1017;
  double t1018;
  double t1038;
  double t1039;
  t981 = Cos(var1[10]);
  t1003 = Cos(var1[11]);
  t1005 = -1.*t1003;
  t1007 = 1. + t1005;
  t1010 = Sin(var1[11]);
  t1015 = Sin(var1[10]);
  t1032 = Cos(var1[9]);
  t1037 = Sin(var1[9]);
  t1008 = -0.19*t1007;
  t1011 = -0.209*t1010;
  t1012 = 0. + t1008 + t1011;
  t1016 = -0.209*t1007;
  t1017 = 0.19*t1010;
  t1018 = 0. + t1016 + t1017;
  t1038 = 0.19*t1015;
  t1039 = 0. + t1038;

  p_output1(0)=0. - 1.*t1015*t1018 - 0.19*(1. - 1.*t981) + t1012*t981 - 0.19*(-1.*t1010*t1015 + t1003*t981) - 0.4165*(-1.*t1003*t1015 - 1.*t1010*t981);
  p_output1(1)=0. + 0.049*(1. - 1.*t1032) + 0.111*t1032 - 1.*t1012*t1015*t1037 - 1.*t1037*t1039 - 1.*t1018*t1037*t981 - 0.4165*(t1010*t1015*t1037 - 1.*t1003*t1037*t981) - 0.19*(-1.*t1003*t1015*t1037 - 1.*t1010*t1037*t981);
  p_output1(2)=0. + t1012*t1015*t1032 + 0.062*t1037 + t1032*t1039 + t1018*t1032*t981 - 0.4165*(-1.*t1010*t1015*t1032 + t1003*t1032*t981) - 0.19*(t1003*t1015*t1032 + t1010*t1032*t981);
}


       
Eigen::Matrix<double,3,1> p_Body_to_HindLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void p_Body_to_HindLeftFoot(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



