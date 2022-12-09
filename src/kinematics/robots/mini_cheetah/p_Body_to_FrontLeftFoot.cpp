/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:03:46 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_Body_to_FrontLeftFoot.h"

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
  double t745;
  double t767;
  double t769;
  double t771;
  double t774;
  double t779;
  double t796;
  double t801;
  double t772;
  double t775;
  double t776;
  double t780;
  double t781;
  double t782;
  double t802;
  double t803;
  t745 = Cos(var1[4]);
  t767 = Cos(var1[5]);
  t769 = -1.*t767;
  t771 = 1. + t769;
  t774 = Sin(var1[5]);
  t779 = Sin(var1[4]);
  t796 = Cos(var1[3]);
  t801 = Sin(var1[3]);
  t772 = 0.19*t771;
  t775 = -0.209*t774;
  t776 = 0. + t772 + t775;
  t780 = -0.209*t771;
  t781 = -0.19*t774;
  t782 = 0. + t780 + t781;
  t802 = -0.19*t779;
  t803 = 0. + t802;

  p_output1(0)=0. + 0.19*(1. - 1.*t745) + t745*t776 - 0.4165*(-1.*t745*t774 - 1.*t767*t779) + 0.19*(t745*t767 - 1.*t774*t779) - 1.*t779*t782;
  p_output1(1)=0. + 0.049*(1. - 1.*t796) + 0.111*t796 - 1.*t776*t779*t801 - 1.*t745*t782*t801 + 0.19*(-1.*t745*t774*t801 - 1.*t767*t779*t801) - 0.4165*(-1.*t745*t767*t801 + t774*t779*t801) - 1.*t801*t803;
  p_output1(2)=0. + t776*t779*t796 + t745*t782*t796 + 0.19*(t745*t774*t796 + t767*t779*t796) - 0.4165*(t745*t767*t796 - 1.*t774*t779*t796) + 0.062*t801 + t796*t803;
}


       
Eigen::Matrix<double,3,1> p_Body_to_FrontLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void p_Body_to_FrontLeftFoot(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



