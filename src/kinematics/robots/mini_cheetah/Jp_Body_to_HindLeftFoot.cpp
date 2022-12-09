/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:03:58 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_Body_to_HindLeftFoot.h"

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
  double t1016;
  double t1022;
  double t1031;
  double t1034;
  double t1038;
  double t1041;
  double t1045;
  double t988;
  double t1024;
  double t1027;
  double t1040;
  double t1042;
  double t1043;
  double t1046;
  double t1047;
  double t1048;
  double t1073;
  double t1074;
  double t1075;
  double t1050;
  double t1051;
  double t1052;
  double t1083;
  double t1084;
  double t1086;
  double t1087;
  double t1088;
  double t1089;
  double t1091;
  double t1092;
  double t1113;
  double t1114;
  double t1116;
  double t1117;
  double t1118;
  double t1098;
  double t1099;
  double t1100;
  double t1101;
  double t1102;
  double t1107;
  double t1108;
  double t1109;
  double t1110;
  double t1111;
  t1016 = Cos(var1[9]);
  t1022 = Sin(var1[10]);
  t1031 = Cos(var1[11]);
  t1034 = -1.*t1031;
  t1038 = 1. + t1034;
  t1041 = Sin(var1[11]);
  t1045 = Cos(var1[10]);
  t988 = Sin(var1[9]);
  t1024 = 0.19*t1022;
  t1027 = 0. + t1024;
  t1040 = -0.19*t1038;
  t1042 = -0.209*t1041;
  t1043 = 0. + t1040 + t1042;
  t1046 = -0.209*t1038;
  t1047 = 0.19*t1041;
  t1048 = 0. + t1046 + t1047;
  t1073 = -1.*t1045*t1031*t988;
  t1074 = t988*t1022*t1041;
  t1075 = t1073 + t1074;
  t1050 = -1.*t1016*t1031*t1022;
  t1051 = -1.*t1016*t1045*t1041;
  t1052 = t1050 + t1051;
  t1083 = -1.*t1031*t1022;
  t1084 = -1.*t1045*t1041;
  t1086 = t1083 + t1084;
  t1087 = -0.19*t1086;
  t1088 = -1.*t1045*t1031;
  t1089 = t1022*t1041;
  t1091 = t1088 + t1089;
  t1092 = -0.4165*t1091;
  t1113 = 0.19*t1031;
  t1114 = t1113 + t1042;
  t1116 = -0.209*t1031;
  t1117 = -0.19*t1041;
  t1118 = t1116 + t1117;
  t1098 = t1031*t988*t1022;
  t1099 = t1045*t988*t1041;
  t1100 = t1098 + t1099;
  t1101 = -0.4165*t1100;
  t1102 = -0.19*t1075;
  t1107 = -0.4165*t1052;
  t1108 = t1016*t1045*t1031;
  t1109 = -1.*t1016*t1022*t1041;
  t1110 = t1108 + t1109;
  t1111 = -0.19*t1110;

  p_output1(0)=0;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=0;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=0;
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
  p_output1(28)=-1.*t1016*t1027 - 1.*t1016*t1022*t1043 - 0.4165*(t1016*t1022*t1041 - 1.*t1016*t1031*t1045) - 1.*t1016*t1045*t1048 - 0.19*t1052 - 0.062*t988;
  p_output1(29)=0.062*t1016 - 0.4165*t1075 - 1.*t1027*t988 - 1.*t1022*t1043*t988 - 1.*t1045*t1048*t988 - 0.19*(-1.*t1022*t1031*t988 - 1.*t1041*t1045*t988);
  p_output1(30)=-0.19*t1022 - 1.*t1022*t1043 - 1.*t1045*t1048 + t1087 + t1092;
  p_output1(31)=t1101 + t1102 - 0.19*t1045*t988 - 1.*t1043*t1045*t988 + t1022*t1048*t988;
  p_output1(32)=0.19*t1016*t1045 + t1016*t1043*t1045 - 1.*t1016*t1022*t1048 + t1107 + t1111;
  p_output1(33)=t1087 + t1092 - 1.*t1022*t1114 + t1045*t1118;
  p_output1(34)=t1101 + t1102 - 1.*t1045*t1114*t988 - 1.*t1022*t1118*t988;
  p_output1(35)=t1107 + t1111 + t1016*t1045*t1114 + t1016*t1022*t1118;
}


       
Eigen::Matrix<double,3,12> Jp_Body_to_HindLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jp_Body_to_HindLeftFoot(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



