/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:51 GMT-05:00
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
  double t1536;
  double t5148;
  double t1123;
  double t5159;
  double t5149;
  double t5164;
  double t5198;
  double t5199;
  double t5169;
  double t5170;
  double t5175;
  double t5144;
  double t5153;
  double t5157;
  double t5200;
  double t5203;
  double t5206;
  double t5209;
  double t5210;
  double t5211;
  double t5160;
  double t5161;
  double t5163;
  double t5176;
  double t5179;
  double t5180;
  double t5165;
  double t5167;
  double t5168;
  double t5181;
  double t5187;
  double t5189;
  t1536 = Cos(var1[11]);
  t5148 = Sin(var1[10]);
  t1123 = Cos(var1[10]);
  t5159 = Sin(var1[9]);
  t5149 = Sin(var1[11]);
  t5164 = Cos(var1[9]);
  t5198 = -1.*t1536;
  t5199 = 1. + t5198;
  t5169 = t1536*t5148;
  t5170 = t1123*t5149;
  t5175 = t5169 + t5170;
  t5144 = t1123*t1536;
  t5153 = -1.*t5148*t5149;
  t5157 = t5144 + t5153;
  t5200 = -0.213*t5199;
  t5203 = -0.1881*t5149;
  t5206 = t5200 + t5203;
  t5209 = -0.1881*t5199;
  t5210 = 0.213*t5149;
  t5211 = t5209 + t5210;
  t5160 = t1536*t5159*t5148;
  t5161 = t1123*t5159*t5149;
  t5163 = t5160 + t5161;
  t5176 = -1.*t1123*t1536*t5159;
  t5179 = t5159*t5148*t5149;
  t5180 = t5176 + t5179;
  t5165 = -1.*t5164*t1536*t5148;
  t5167 = -1.*t5164*t1123*t5149;
  t5168 = t5165 + t5167;
  t5181 = t5164*t1123*t1536;
  t5187 = -1.*t5164*t5148*t5149;
  t5189 = t5181 + t5187;

  p_output1(0)=t5157;
  p_output1(1)=t5163;
  p_output1(2)=t5168;
  p_output1(3)=0;
  p_output1(4)=0;
  p_output1(5)=t5164;
  p_output1(6)=t5159;
  p_output1(7)=0;
  p_output1(8)=t5175;
  p_output1(9)=t5180;
  p_output1(10)=t5189;
  p_output1(11)=0;
  p_output1(12)=0. - 0.1881*(1. - 1.*t1123) - 0.1881*t5157 - 0.4205*t5175 + t5148*t5206 + t1123*t5211;
  p_output1(13)=0. + 0.1881*t5148*t5159 - 0.1881*t5163 + 0.04675*(1. - 1.*t5164) + 0.12675*t5164 - 0.4205*t5180 - 1.*t1123*t5159*t5206 + t5148*t5159*t5211;
  p_output1(14)=0. + 0.08*t5159 - 0.1881*t5148*t5164 - 0.1881*t5168 - 0.4205*t5189 + t1123*t5164*t5206 - 1.*t5148*t5164*t5211;
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



