/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:03:52 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_Body_to_FrontRightFoot.h"

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
  double t863;
  double t885;
  double t887;
  double t889;
  double t892;
  double t897;
  double t914;
  double t919;
  double t890;
  double t893;
  double t894;
  double t898;
  double t899;
  double t900;
  double t920;
  double t921;
  t863 = Cos(var1[1]);
  t885 = Cos(var1[2]);
  t887 = -1.*t885;
  t889 = 1. + t887;
  t892 = Sin(var1[2]);
  t897 = Sin(var1[1]);
  t914 = Cos(var1[0]);
  t919 = Sin(var1[0]);
  t890 = 0.19*t889;
  t893 = -0.209*t892;
  t894 = 0. + t890 + t893;
  t898 = -0.209*t889;
  t899 = -0.19*t892;
  t900 = 0. + t898 + t899;
  t920 = -0.19*t897;
  t921 = 0. + t920;

  p_output1(0)=0. + 0.19*(1. - 1.*t863) + t863*t894 - 0.4165*(-1.*t863*t892 - 1.*t885*t897) + 0.19*(t863*t885 - 1.*t892*t897) - 1.*t897*t900;
  p_output1(1)=0. - 0.049*(1. - 1.*t914) - 0.111*t914 - 1.*t894*t897*t919 - 1.*t863*t900*t919 + 0.19*(-1.*t863*t892*t919 - 1.*t885*t897*t919) - 0.4165*(-1.*t863*t885*t919 + t892*t897*t919) - 1.*t919*t921;
  p_output1(2)=0. + t894*t897*t914 + t863*t900*t914 + 0.19*(t863*t892*t914 + t885*t897*t914) - 0.4165*(t863*t885*t914 - 1.*t892*t897*t914) - 0.062*t919 + t914*t921;
}


       
Eigen::Matrix<double,3,1> p_Body_to_FrontRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void p_Body_to_FrontRightFoot(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



