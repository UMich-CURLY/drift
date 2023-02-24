/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:49 GMT-05:00
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
  double t1188;
  double t5030;
  double t586;
  double t5062;
  double t5055;
  double t5070;
  double t5105;
  double t5106;
  double t5076;
  double t5077;
  double t5080;
  double t1358;
  double t5060;
  double t5061;
  double t5108;
  double t5111;
  double t5113;
  double t5116;
  double t5117;
  double t5118;
  double t5067;
  double t5068;
  double t5069;
  double t5081;
  double t5085;
  double t5086;
  double t5072;
  double t5073;
  double t5074;
  double t5088;
  double t5093;
  double t5095;
  t1188 = Cos(var1[2]);
  t5030 = Sin(var1[1]);
  t586 = Cos(var1[1]);
  t5062 = Sin(var1[0]);
  t5055 = Sin(var1[2]);
  t5070 = Cos(var1[0]);
  t5105 = -1.*t1188;
  t5106 = 1. + t5105;
  t5076 = t1188*t5030;
  t5077 = t586*t5055;
  t5080 = t5076 + t5077;
  t1358 = t586*t1188;
  t5060 = -1.*t5030*t5055;
  t5061 = t1358 + t5060;
  t5108 = -0.213*t5106;
  t5111 = 0.1881*t5055;
  t5113 = t5108 + t5111;
  t5116 = 0.1881*t5106;
  t5117 = 0.213*t5055;
  t5118 = t5116 + t5117;
  t5067 = t1188*t5062*t5030;
  t5068 = t586*t5062*t5055;
  t5069 = t5067 + t5068;
  t5081 = -1.*t586*t1188*t5062;
  t5085 = t5062*t5030*t5055;
  t5086 = t5081 + t5085;
  t5072 = -1.*t5070*t1188*t5030;
  t5073 = -1.*t5070*t586*t5055;
  t5074 = t5072 + t5073;
  t5088 = t5070*t586*t1188;
  t5093 = -1.*t5070*t5030*t5055;
  t5095 = t5088 + t5093;

  p_output1(0)=t5061;
  p_output1(1)=t5069;
  p_output1(2)=t5074;
  p_output1(3)=0;
  p_output1(4)=0;
  p_output1(5)=t5070;
  p_output1(6)=t5062;
  p_output1(7)=0;
  p_output1(8)=t5080;
  p_output1(9)=t5086;
  p_output1(10)=t5095;
  p_output1(11)=0;
  p_output1(12)=0. + 0.1881*t5061 - 0.4205*t5080 + t5030*t5113 + 0.1881*(1. - 1.*t586) + t5118*t586;
  p_output1(13)=0. - 0.1881*t5030*t5062 + 0.1881*t5069 - 0.04675*(1. - 1.*t5070) - 0.12675*t5070 - 0.4205*t5086 + t5030*t5062*t5118 - 1.*t5062*t5113*t586;
  p_output1(14)=0. - 0.08*t5062 + 0.1881*t5030*t5070 + 0.1881*t5074 - 0.4205*t5095 - 1.*t5030*t5070*t5118 + t5070*t5113*t586;
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



