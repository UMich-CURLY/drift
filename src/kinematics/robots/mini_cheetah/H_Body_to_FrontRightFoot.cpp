/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:03:49 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_Body_to_FrontRightFoot.h"

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
  double t810;
  double t820;
  double t794;
  double t824;
  double t821;
  double t836;
  double t872;
  double t874;
  double t853;
  double t857;
  double t817;
  double t822;
  double t875;
  double t877;
  double t879;
  double t881;
  double t883;
  double t884;
  double t831;
  double t834;
  double t860;
  double t862;
  double t895;
  double t896;
  double t839;
  double t840;
  double t864;
  double t867;
  t810 = Cos(var1[2]);
  t820 = Sin(var1[1]);
  t794 = Cos(var1[1]);
  t824 = Sin(var1[0]);
  t821 = Sin(var1[2]);
  t836 = Cos(var1[0]);
  t872 = -1.*t810;
  t874 = 1. + t872;
  t853 = -1.*t810*t820;
  t857 = -1.*t794*t821;
  t817 = t794*t810;
  t822 = -1.*t820*t821;
  t875 = 0.19*t874;
  t877 = -0.209*t821;
  t879 = 0. + t875 + t877;
  t881 = -0.209*t874;
  t883 = -0.19*t821;
  t884 = 0. + t881 + t883;
  t831 = -1.*t810*t824*t820;
  t834 = -1.*t794*t824*t821;
  t860 = -1.*t794*t810*t824;
  t862 = t824*t820*t821;
  t895 = -0.19*t820;
  t896 = 0. + t895;
  t839 = t836*t810*t820;
  t840 = t836*t794*t821;
  t864 = t836*t794*t810;
  t867 = -1.*t836*t820*t821;

  p_output1(0)=0. + t817 + t822;
  p_output1(1)=0. + t831 + t834;
  p_output1(2)=0. + t839 + t840;
  p_output1(3)=0.;
  p_output1(4)=0.;
  p_output1(5)=0. + t836;
  p_output1(6)=0. + t824;
  p_output1(7)=0.;
  p_output1(8)=0. + t853 + t857;
  p_output1(9)=0. + t860 + t862;
  p_output1(10)=0. + t864 + t867;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.19*(1. - 1.*t794) + 0.19*(t817 + t822) - 0.4165*(t853 + t857) + t794*t879 - 1.*t820*t884;
  p_output1(13)=0. + 0.19*(t831 + t834) - 0.049*(1. - 1.*t836) - 0.111*t836 - 0.4165*(t860 + t862) - 1.*t820*t824*t879 - 1.*t794*t824*t884 - 1.*t824*t896;
  p_output1(14)=0. - 0.062*t824 + 0.19*(t839 + t840) - 0.4165*(t864 + t867) + t820*t836*t879 + t794*t836*t884 + t836*t896;
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_Body_to_FrontRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void H_Body_to_FrontRightFoot(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



