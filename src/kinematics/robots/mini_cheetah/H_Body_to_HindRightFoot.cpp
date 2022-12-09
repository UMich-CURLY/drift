/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:04:00 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_Body_to_HindRightFoot.h"

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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  double t1046;
  double t1056;
  double t1030;
  double t1060;
  double t1057;
  double t1072;
  double t1108;
  double t1110;
  double t1089;
  double t1093;
  double t1053;
  double t1058;
  double t1111;
  double t1113;
  double t1115;
  double t1117;
  double t1119;
  double t1120;
  double t1067;
  double t1070;
  double t1096;
  double t1098;
  double t1131;
  double t1132;
  double t1075;
  double t1076;
  double t1100;
  double t1103;
  t1046 = Cos(var1[8]);
  t1056 = Sin(var1[7]);
  t1030 = Cos(var1[7]);
  t1060 = Sin(var1[6]);
  t1057 = Sin(var1[8]);
  t1072 = Cos(var1[6]);
  t1108 = -1.*t1046;
  t1110 = 1. + t1108;
  t1089 = -1.*t1046*t1056;
  t1093 = -1.*t1030*t1057;
  t1053 = t1030*t1046;
  t1058 = -1.*t1056*t1057;
  t1111 = -0.19*t1110;
  t1113 = -0.209*t1057;
  t1115 = 0. + t1111 + t1113;
  t1117 = -0.209*t1110;
  t1119 = 0.19*t1057;
  t1120 = 0. + t1117 + t1119;
  t1067 = -1.*t1046*t1060*t1056;
  t1070 = -1.*t1030*t1060*t1057;
  t1096 = -1.*t1030*t1046*t1060;
  t1098 = t1060*t1056*t1057;
  t1131 = 0.19*t1056;
  t1132 = 0. + t1131;
  t1075 = t1072*t1046*t1056;
  t1076 = t1072*t1030*t1057;
  t1100 = t1072*t1030*t1046;
  t1103 = -1.*t1072*t1056*t1057;

  p_output1(0)=0. + t1053 + t1058;
  p_output1(1)=0. + t1067 + t1070;
  p_output1(2)=0. + t1075 + t1076;
  p_output1(3)=0.;
  p_output1(4)=0.;
  p_output1(5)=0. + t1072;
  p_output1(6)=0. + t1060;
  p_output1(7)=0.;
  p_output1(8)=0. + t1089 + t1093;
  p_output1(9)=0. + t1096 + t1098;
  p_output1(10)=0. + t1100 + t1103;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.19*(1. - 1.*t1030) - 0.19*(t1053 + t1058) - 0.4165*(t1089 + t1093) + t1030*t1115 - 1.*t1056*t1120;
  p_output1(13)=0. - 0.19*(t1067 + t1070) - 0.049*(1. - 1.*t1072) - 0.111*t1072 - 0.4165*(t1096 + t1098) - 1.*t1056*t1060*t1115 - 1.*t1030*t1060*t1120 - 1.*t1060*t1132;
  p_output1(14)=0. - 0.062*t1060 - 0.19*(t1075 + t1076) - 0.4165*(t1100 + t1103) + t1056*t1072*t1115 + t1030*t1072*t1120 + t1072*t1132;
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_Body_to_HindRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void H_Body_to_HindRightFoot(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



