/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:51 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_Body_to_FrontRightFoot.h"

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
  double t1127;
  double t1519;
  double t5134;
  double t5136;
  double t5139;
  double t5141;
  double t5125;
  double t373;
  double t5140;
  double t5144;
  double t5146;
  double t5148;
  double t5149;
  double t5152;
  double t5167;
  double t5168;
  double t5169;
  double t5158;
  double t5159;
  double t5160;
  double t5182;
  double t5183;
  double t5185;
  double t5187;
  double t5189;
  double t5190;
  double t5191;
  double t5192;
  double t5218;
  double t5219;
  double t5220;
  double t5222;
  double t5223;
  double t5202;
  double t5203;
  double t5205;
  double t5206;
  double t5207;
  double t5212;
  double t5213;
  double t5214;
  double t5215;
  double t5216;
  t1127 = Cos(var1[0]);
  t1519 = Sin(var1[1]);
  t5134 = Cos(var1[2]);
  t5136 = -1.*t5134;
  t5139 = 1. + t5136;
  t5141 = Sin(var1[2]);
  t5125 = Cos(var1[1]);
  t373 = Sin(var1[0]);
  t5140 = -0.213*t5139;
  t5144 = 0.1881*t5141;
  t5146 = t5140 + t5144;
  t5148 = 0.1881*t5139;
  t5149 = 0.213*t5141;
  t5152 = t5148 + t5149;
  t5167 = t5134*t373*t1519;
  t5168 = t5125*t373*t5141;
  t5169 = t5167 + t5168;
  t5158 = -1.*t1127*t5125*t5134;
  t5159 = t1127*t1519*t5141;
  t5160 = t5158 + t5159;
  t5182 = -1.*t5134*t1519;
  t5183 = -1.*t5125*t5141;
  t5185 = t5182 + t5183;
  t5187 = 0.1881*t5185;
  t5189 = t5125*t5134;
  t5190 = -1.*t1519*t5141;
  t5191 = t5189 + t5190;
  t5192 = -0.4205*t5191;
  t5218 = 0.1881*t5134;
  t5219 = -0.213*t5141;
  t5220 = t5218 + t5219;
  t5222 = 0.213*t5134;
  t5223 = t5222 + t5144;
  t5202 = -0.4205*t5169;
  t5203 = t5125*t5134*t373;
  t5205 = -1.*t373*t1519*t5141;
  t5206 = t5203 + t5205;
  t5207 = 0.1881*t5206;
  t5212 = -1.*t1127*t5134*t1519;
  t5213 = -1.*t1127*t5125*t5141;
  t5214 = t5212 + t5213;
  t5215 = -0.4205*t5214;
  t5216 = 0.1881*t5160;

  p_output1(0)=0;
  p_output1(1)=-0.1881*t1127*t1519 + 0.08*t373 + 0.1881*(t1127*t1519*t5134 + t1127*t5125*t5141) - 1.*t1127*t5125*t5146 + t1127*t1519*t5152 - 0.4205*t5160;
  p_output1(2)=-0.08*t1127 - 0.1881*t1519*t373 - 0.4205*(-1.*t373*t5125*t5134 + t1519*t373*t5141) - 1.*t373*t5125*t5146 + t1519*t373*t5152 + 0.1881*t5169;
  p_output1(3)=0.1881*t1519 + t5125*t5146 - 1.*t1519*t5152 + t5187 + t5192;
  p_output1(4)=-0.1881*t373*t5125 + t1519*t373*t5146 + t373*t5125*t5152 + t5202 + t5207;
  p_output1(5)=0.1881*t1127*t5125 - 1.*t1127*t1519*t5146 - 1.*t1127*t5125*t5152 + t5215 + t5216;
  p_output1(6)=t5187 + t5192 + t1519*t5220 + t5125*t5223;
  p_output1(7)=t5202 + t5207 - 1.*t373*t5125*t5220 + t1519*t373*t5223;
  p_output1(8)=t5215 + t5216 + t1127*t5125*t5220 - 1.*t1127*t1519*t5223;
  p_output1(9)=0;
  p_output1(10)=0;
  p_output1(11)=0;
  p_output1(12)=0;
  p_output1(13)=0;
  p_output1(14)=0;
  p_output1(15)=0;
  p_output1(16)=0;
  p_output1(17)=0;
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=0;
  p_output1(34)=0;
  p_output1(35)=0;
}


       
Eigen::Matrix<double,3,12> Jp_Body_to_FrontRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jp_Body_to_FrontRightFoot(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



