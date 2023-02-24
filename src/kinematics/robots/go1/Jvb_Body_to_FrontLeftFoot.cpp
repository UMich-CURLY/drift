/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:57:14 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jvb_Body_to_FrontLeftFoot.h"

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
static void output1(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  double t971;
  double t5972;
  double t5974;
  double t5976;
  double t5989;
  double t5991;
  double t5992;
  double t5999;
  double t6002;
  double t5973;
  double t5983;
  double t5984;
  double t6003;
  double t6004;
  double t6006;
  double t6011;
  double t6012;
  double t6013;
  double t6014;
  double t6024;
  double t6025;
  double t6026;
  double t6018;
  double t6019;
  double t6020;
  double t6021;
  double t6022;
  double t6023;
  double t6009;
  double t6010;
  double t6015;
  double t6016;
  t971 = Cos(var1[5]);
  t5972 = Sin(var1[4]);
  t5974 = Cos(var1[4]);
  t5976 = Sin(var1[5]);
  t5989 = t971*t5972;
  t5991 = t5974*t5976;
  t5992 = t5989 + t5991;
  t5999 = -1.*t971;
  t6002 = 1. + t5999;
  t5973 = -1.*t971*t5972;
  t5983 = -1.*t5974*t5976;
  t5984 = t5973 + t5983;
  t6003 = -0.213*t6002;
  t6004 = -0.4205*t971;
  t6006 = t6003 + t6004;
  t6011 = 0.1881*t6002;
  t6012 = 0.1881*t971;
  t6013 = -0.2075*t5976;
  t6014 = t6011 + t6012 + t6013;
  t6024 = t5974*t971;
  t6025 = -1.*t5972*t5976;
  t6026 = t6024 + t6025;
  t6018 = -1.*t5974;
  t6019 = 1. + t6018;
  t6020 = 0.1881*t6019;
  t6021 = t6006*t5972;
  t6022 = t5974*t6014;
  t6023 = t6020 + t6021 + t6022;
  t6009 = t5974*t6006;
  t6010 = 0.1881*t5972;
  t6015 = -1.*t5972*t6014;
  t6016 = t6009 + t6010 + t6015;

  p_output1(0)=0;
  p_output1(1)=0.;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=0.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0.;
  p_output1(8)=0;
  p_output1(9)=-0.04675*t5984 - 0.12675*t5992;
  p_output1(10)=0. + t6026*(-1.*t5992*t6023 - 1.*t6016*t6026) + t5992*(t5984*t6016 + t6023*t6026);
  p_output1(11)=0.08*t6026;
  p_output1(12)=-0.1881*t5976 + t5976*t6014 + t6006*t971;
  p_output1(13)=0.;
  p_output1(14)=t5976*t6006 + t6012 - 1.*t6014*t971;
  p_output1(15)=-0.2075;
  p_output1(16)=0.;
  p_output1(17)=0;
  p_output1(18)=0;
  p_output1(19)=0.;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0.;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0.;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0.;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0.;
  p_output1(32)=0;
  p_output1(33)=0;
  p_output1(34)=0.;
  p_output1(35)=0;
}


       
Eigen::Matrix<double,3,12> Jvb_Body_to_FrontLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jvb_Body_to_FrontLeftFoot(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



