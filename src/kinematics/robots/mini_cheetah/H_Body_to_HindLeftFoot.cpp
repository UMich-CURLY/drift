/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:03:54 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_Body_to_HindLeftFoot.h"

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
  double t928;
  double t938;
  double t912;
  double t942;
  double t939;
  double t954;
  double t990;
  double t992;
  double t971;
  double t975;
  double t935;
  double t940;
  double t993;
  double t995;
  double t997;
  double t999;
  double t1001;
  double t1002;
  double t949;
  double t952;
  double t978;
  double t980;
  double t1013;
  double t1014;
  double t957;
  double t958;
  double t982;
  double t985;
  t928 = Cos(var1[11]);
  t938 = Sin(var1[10]);
  t912 = Cos(var1[10]);
  t942 = Sin(var1[9]);
  t939 = Sin(var1[11]);
  t954 = Cos(var1[9]);
  t990 = -1.*t928;
  t992 = 1. + t990;
  t971 = -1.*t928*t938;
  t975 = -1.*t912*t939;
  t935 = t912*t928;
  t940 = -1.*t938*t939;
  t993 = -0.19*t992;
  t995 = -0.209*t939;
  t997 = 0. + t993 + t995;
  t999 = -0.209*t992;
  t1001 = 0.19*t939;
  t1002 = 0. + t999 + t1001;
  t949 = -1.*t928*t942*t938;
  t952 = -1.*t912*t942*t939;
  t978 = -1.*t912*t928*t942;
  t980 = t942*t938*t939;
  t1013 = 0.19*t938;
  t1014 = 0. + t1013;
  t957 = t954*t928*t938;
  t958 = t954*t912*t939;
  t982 = t954*t912*t928;
  t985 = -1.*t954*t938*t939;

  p_output1(0)=0. + t935 + t940;
  p_output1(1)=0. + t949 + t952;
  p_output1(2)=0. + t957 + t958;
  p_output1(3)=0.;
  p_output1(4)=0.;
  p_output1(5)=0. + t954;
  p_output1(6)=0. + t942;
  p_output1(7)=0.;
  p_output1(8)=0. + t971 + t975;
  p_output1(9)=0. + t978 + t980;
  p_output1(10)=0. + t982 + t985;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.19*(1. - 1.*t912) - 1.*t1002*t938 - 0.19*(t935 + t940) - 0.4165*(t971 + t975) + t912*t997;
  p_output1(13)=0. - 1.*t1014*t942 - 1.*t1002*t912*t942 - 0.19*(t949 + t952) + 0.049*(1. - 1.*t954) + 0.111*t954 - 0.4165*(t978 + t980) - 1.*t938*t942*t997;
  p_output1(14)=0. + 0.062*t942 + t1014*t954 + t1002*t912*t954 - 0.19*(t957 + t958) - 0.4165*(t982 + t985) + t938*t954*t997;
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_Body_to_HindLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void H_Body_to_HindLeftFoot(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



