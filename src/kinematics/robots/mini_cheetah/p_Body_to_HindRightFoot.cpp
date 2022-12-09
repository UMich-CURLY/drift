/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:04:03 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_Body_to_HindRightFoot.h"

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
  double t1099;
  double t1121;
  double t1123;
  double t1125;
  double t1128;
  double t1133;
  double t1150;
  double t1155;
  double t1126;
  double t1129;
  double t1130;
  double t1134;
  double t1135;
  double t1136;
  double t1156;
  double t1157;
  t1099 = Cos(var1[7]);
  t1121 = Cos(var1[8]);
  t1123 = -1.*t1121;
  t1125 = 1. + t1123;
  t1128 = Sin(var1[8]);
  t1133 = Sin(var1[7]);
  t1150 = Cos(var1[6]);
  t1155 = Sin(var1[6]);
  t1126 = -0.19*t1125;
  t1129 = -0.209*t1128;
  t1130 = 0. + t1126 + t1129;
  t1134 = -0.209*t1125;
  t1135 = 0.19*t1128;
  t1136 = 0. + t1134 + t1135;
  t1156 = 0.19*t1133;
  t1157 = 0. + t1156;

  p_output1(0)=0. - 0.19*(1. - 1.*t1099) + t1099*t1130 - 0.4165*(-1.*t1099*t1128 - 1.*t1121*t1133) - 0.19*(t1099*t1121 - 1.*t1128*t1133) - 1.*t1133*t1136;
  p_output1(1)=0. - 0.049*(1. - 1.*t1150) - 0.111*t1150 - 1.*t1130*t1133*t1155 - 1.*t1099*t1136*t1155 - 0.19*(-1.*t1099*t1128*t1155 - 1.*t1121*t1133*t1155) - 0.4165*(-1.*t1099*t1121*t1155 + t1128*t1133*t1155) - 1.*t1155*t1157;
  p_output1(2)=0. + t1130*t1133*t1150 + t1099*t1136*t1150 - 0.19*(t1099*t1128*t1150 + t1121*t1133*t1150) - 0.4165*(t1099*t1121*t1150 - 1.*t1128*t1133*t1150) - 0.062*t1155 + t1150*t1157;
}


       
Eigen::Matrix<double,3,1> p_Body_to_HindRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void p_Body_to_HindRightFoot(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



