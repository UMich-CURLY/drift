/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:03:53 GMT-05:00
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
  double t898;
  double t904;
  double t913;
  double t916;
  double t920;
  double t923;
  double t927;
  double t870;
  double t906;
  double t909;
  double t922;
  double t924;
  double t925;
  double t928;
  double t929;
  double t930;
  double t955;
  double t956;
  double t957;
  double t932;
  double t933;
  double t934;
  double t965;
  double t966;
  double t968;
  double t969;
  double t970;
  double t971;
  double t973;
  double t974;
  double t995;
  double t996;
  double t998;
  double t999;
  double t1000;
  double t980;
  double t981;
  double t982;
  double t983;
  double t984;
  double t989;
  double t990;
  double t991;
  double t992;
  double t993;
  t898 = Cos(var1[0]);
  t904 = Sin(var1[1]);
  t913 = Cos(var1[2]);
  t916 = -1.*t913;
  t920 = 1. + t916;
  t923 = Sin(var1[2]);
  t927 = Cos(var1[1]);
  t870 = Sin(var1[0]);
  t906 = -0.19*t904;
  t909 = 0. + t906;
  t922 = 0.19*t920;
  t924 = -0.209*t923;
  t925 = 0. + t922 + t924;
  t928 = -0.209*t920;
  t929 = -0.19*t923;
  t930 = 0. + t928 + t929;
  t955 = -1.*t927*t913*t870;
  t956 = t870*t904*t923;
  t957 = t955 + t956;
  t932 = -1.*t898*t913*t904;
  t933 = -1.*t898*t927*t923;
  t934 = t932 + t933;
  t965 = -1.*t913*t904;
  t966 = -1.*t927*t923;
  t968 = t965 + t966;
  t969 = 0.19*t968;
  t970 = -1.*t927*t913;
  t971 = t904*t923;
  t973 = t970 + t971;
  t974 = -0.4165*t973;
  t995 = -0.19*t913;
  t996 = t995 + t924;
  t998 = -0.209*t913;
  t999 = 0.19*t923;
  t1000 = t998 + t999;
  t980 = t913*t870*t904;
  t981 = t927*t870*t923;
  t982 = t980 + t981;
  t983 = -0.4165*t982;
  t984 = 0.19*t957;
  t989 = -0.4165*t934;
  t990 = t898*t927*t913;
  t991 = -1.*t898*t904*t923;
  t992 = t990 + t991;
  t993 = 0.19*t992;

  p_output1(0)=0;
  p_output1(1)=0.062*t870 - 1.*t898*t909 - 1.*t898*t904*t925 - 0.4165*(t898*t904*t923 - 1.*t898*t913*t927) - 1.*t898*t927*t930 + 0.19*t934;
  p_output1(2)=-0.062*t898 - 1.*t870*t909 - 1.*t870*t904*t925 + 0.19*(-1.*t870*t904*t913 - 1.*t870*t923*t927) - 1.*t870*t927*t930 - 0.4165*t957;
  p_output1(3)=0.19*t904 - 1.*t904*t925 - 1.*t927*t930 + t969 + t974;
  p_output1(4)=0.19*t870*t927 - 1.*t870*t925*t927 + t870*t904*t930 + t983 + t984;
  p_output1(5)=-0.19*t898*t927 + t898*t925*t927 - 1.*t898*t904*t930 + t989 + t993;
  p_output1(6)=t1000*t927 + t969 + t974 - 1.*t904*t996;
  p_output1(7)=-1.*t1000*t870*t904 + t983 + t984 - 1.*t870*t927*t996;
  p_output1(8)=t1000*t898*t904 + t989 + t993 + t898*t927*t996;
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



