/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:03:48 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_Body_to_FrontLeftFoot.h"

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
  double t780;
  double t786;
  double t795;
  double t798;
  double t802;
  double t805;
  double t809;
  double t752;
  double t788;
  double t791;
  double t804;
  double t806;
  double t807;
  double t810;
  double t811;
  double t812;
  double t837;
  double t838;
  double t839;
  double t814;
  double t815;
  double t816;
  double t847;
  double t848;
  double t850;
  double t851;
  double t852;
  double t853;
  double t855;
  double t856;
  double t877;
  double t878;
  double t880;
  double t881;
  double t882;
  double t862;
  double t863;
  double t864;
  double t865;
  double t866;
  double t871;
  double t872;
  double t873;
  double t874;
  double t875;
  t780 = Cos(var1[3]);
  t786 = Sin(var1[4]);
  t795 = Cos(var1[5]);
  t798 = -1.*t795;
  t802 = 1. + t798;
  t805 = Sin(var1[5]);
  t809 = Cos(var1[4]);
  t752 = Sin(var1[3]);
  t788 = -0.19*t786;
  t791 = 0. + t788;
  t804 = 0.19*t802;
  t806 = -0.209*t805;
  t807 = 0. + t804 + t806;
  t810 = -0.209*t802;
  t811 = -0.19*t805;
  t812 = 0. + t810 + t811;
  t837 = -1.*t809*t795*t752;
  t838 = t752*t786*t805;
  t839 = t837 + t838;
  t814 = -1.*t780*t795*t786;
  t815 = -1.*t780*t809*t805;
  t816 = t814 + t815;
  t847 = -1.*t795*t786;
  t848 = -1.*t809*t805;
  t850 = t847 + t848;
  t851 = 0.19*t850;
  t852 = -1.*t809*t795;
  t853 = t786*t805;
  t855 = t852 + t853;
  t856 = -0.4165*t855;
  t877 = -0.19*t795;
  t878 = t877 + t806;
  t880 = -0.209*t795;
  t881 = 0.19*t805;
  t882 = t880 + t881;
  t862 = t795*t752*t786;
  t863 = t809*t752*t805;
  t864 = t862 + t863;
  t865 = -0.4165*t864;
  t866 = 0.19*t839;
  t871 = -0.4165*t816;
  t872 = t780*t809*t795;
  t873 = -1.*t780*t786*t805;
  t874 = t872 + t873;
  t875 = 0.19*t874;

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
  p_output1(10)=-0.062*t752 - 1.*t780*t791 - 1.*t780*t786*t807 - 0.4165*(t780*t786*t805 - 1.*t780*t795*t809) - 1.*t780*t809*t812 + 0.19*t816;
  p_output1(11)=0.062*t780 - 1.*t752*t791 - 1.*t752*t786*t807 + 0.19*(-1.*t752*t786*t795 - 1.*t752*t805*t809) - 1.*t752*t809*t812 - 0.4165*t839;
  p_output1(12)=0.19*t786 - 1.*t786*t807 - 1.*t809*t812 + t851 + t856;
  p_output1(13)=0.19*t752*t809 - 1.*t752*t807*t809 + t752*t786*t812 + t865 + t866;
  p_output1(14)=-0.19*t780*t809 + t780*t807*t809 - 1.*t780*t786*t812 + t871 + t875;
  p_output1(15)=t851 + t856 - 1.*t786*t878 + t809*t882;
  p_output1(16)=t865 + t866 - 1.*t752*t809*t878 - 1.*t752*t786*t882;
  p_output1(17)=t871 + t875 + t780*t809*t878 + t780*t786*t882;
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


       
Eigen::Matrix<double,3,12> Jp_Body_to_FrontLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jp_Body_to_FrontLeftFoot(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



