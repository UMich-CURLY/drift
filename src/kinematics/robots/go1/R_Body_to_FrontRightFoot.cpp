/*
 * Automatically Generated from Mathematica.
 * Fri 10 Feb 2023 15:56:49 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "kinematics/robots/go1/R_Body_to_FrontRightFoot.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  double t5068;
  double t5076;
  double t3021;
  double t5088;
  double t5077;
  double t5104;
  t5068 = Cos(var1[2]);
  t5076 = Sin(var1[1]);
  t3021 = Cos(var1[1]);
  t5088 = Sin(var1[0]);
  t5077 = Sin(var1[2]);
  t5104 = Cos(var1[0]);

  p_output1(0)=t3021*t5068 - 1.*t5076*t5077;
  p_output1(1)=t5068*t5076*t5088 + t3021*t5077*t5088;
  p_output1(2)=-1.*t5068*t5076*t5104 - 1.*t3021*t5077*t5104;
  p_output1(3)=0;
  p_output1(4)=t5104;
  p_output1(5)=t5088;
  p_output1(6)=t5068*t5076 + t3021*t5077;
  p_output1(7)=-1.*t3021*t5068*t5088 + t5076*t5077*t5088;
  p_output1(8)=t3021*t5068*t5104 - 1.*t5076*t5077*t5104;
}


       
Eigen::Matrix<double,3,3> R_Body_to_FrontRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void R_Body_to_FrontRightFoot(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



