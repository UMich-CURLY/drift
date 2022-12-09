/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:04:04 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_Body_to_HindRightFoot.h"

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
  double t1134;
  double t1140;
  double t1149;
  double t1152;
  double t1156;
  double t1159;
  double t1163;
  double t1106;
  double t1142;
  double t1145;
  double t1158;
  double t1160;
  double t1161;
  double t1164;
  double t1165;
  double t1166;
  double t1191;
  double t1192;
  double t1193;
  double t1168;
  double t1169;
  double t1170;
  double t1201;
  double t1202;
  double t1204;
  double t1205;
  double t1206;
  double t1207;
  double t1209;
  double t1210;
  double t1231;
  double t1232;
  double t1234;
  double t1235;
  double t1236;
  double t1216;
  double t1217;
  double t1218;
  double t1219;
  double t1220;
  double t1225;
  double t1226;
  double t1227;
  double t1228;
  double t1229;
  t1134 = Cos(var1[6]);
  t1140 = Sin(var1[7]);
  t1149 = Cos(var1[8]);
  t1152 = -1.*t1149;
  t1156 = 1. + t1152;
  t1159 = Sin(var1[8]);
  t1163 = Cos(var1[7]);
  t1106 = Sin(var1[6]);
  t1142 = 0.19*t1140;
  t1145 = 0. + t1142;
  t1158 = -0.19*t1156;
  t1160 = -0.209*t1159;
  t1161 = 0. + t1158 + t1160;
  t1164 = -0.209*t1156;
  t1165 = 0.19*t1159;
  t1166 = 0. + t1164 + t1165;
  t1191 = -1.*t1163*t1149*t1106;
  t1192 = t1106*t1140*t1159;
  t1193 = t1191 + t1192;
  t1168 = -1.*t1134*t1149*t1140;
  t1169 = -1.*t1134*t1163*t1159;
  t1170 = t1168 + t1169;
  t1201 = -1.*t1149*t1140;
  t1202 = -1.*t1163*t1159;
  t1204 = t1201 + t1202;
  t1205 = -0.19*t1204;
  t1206 = -1.*t1163*t1149;
  t1207 = t1140*t1159;
  t1209 = t1206 + t1207;
  t1210 = -0.4165*t1209;
  t1231 = 0.19*t1149;
  t1232 = t1231 + t1160;
  t1234 = -0.209*t1149;
  t1235 = -0.19*t1159;
  t1236 = t1234 + t1235;
  t1216 = t1149*t1106*t1140;
  t1217 = t1163*t1106*t1159;
  t1218 = t1216 + t1217;
  t1219 = -0.4165*t1218;
  t1220 = -0.19*t1193;
  t1225 = -0.4165*t1170;
  t1226 = t1134*t1163*t1149;
  t1227 = -1.*t1134*t1140*t1159;
  t1228 = t1226 + t1227;
  t1229 = -0.19*t1228;

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
  p_output1(19)=0.062*t1106 - 1.*t1134*t1145 - 1.*t1134*t1140*t1161 - 0.4165*(t1134*t1140*t1159 - 1.*t1134*t1149*t1163) - 1.*t1134*t1163*t1166 - 0.19*t1170;
  p_output1(20)=-0.062*t1134 - 1.*t1106*t1145 - 1.*t1106*t1140*t1161 - 0.19*(-1.*t1106*t1140*t1149 - 1.*t1106*t1159*t1163) - 1.*t1106*t1163*t1166 - 0.4165*t1193;
  p_output1(21)=-0.19*t1140 - 1.*t1140*t1161 - 1.*t1163*t1166 + t1205 + t1210;
  p_output1(22)=-0.19*t1106*t1163 - 1.*t1106*t1161*t1163 + t1106*t1140*t1166 + t1219 + t1220;
  p_output1(23)=0.19*t1134*t1163 + t1134*t1161*t1163 - 1.*t1134*t1140*t1166 + t1225 + t1229;
  p_output1(24)=t1205 + t1210 - 1.*t1140*t1232 + t1163*t1236;
  p_output1(25)=t1219 + t1220 - 1.*t1106*t1163*t1232 - 1.*t1106*t1140*t1236;
  p_output1(26)=t1225 + t1229 + t1134*t1163*t1232 + t1134*t1140*t1236;
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


       
Eigen::Matrix<double,3,12> Jp_Body_to_HindRightFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jp_Body_to_HindRightFoot(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



