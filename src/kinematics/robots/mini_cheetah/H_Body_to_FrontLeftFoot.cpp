/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:03:44 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_Body_to_FrontLeftFoot.h"

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
  double t682;
  double t697;
  double t668;
  double t703;
  double t698;
  double t718;
  double t754;
  double t756;
  double t735;
  double t739;
  double t694;
  double t701;
  double t757;
  double t759;
  double t761;
  double t763;
  double t765;
  double t766;
  double t711;
  double t715;
  double t742;
  double t744;
  double t777;
  double t778;
  double t721;
  double t722;
  double t746;
  double t749;
  t682 = Cos(var1[5]);
  t697 = Sin(var1[4]);
  t668 = Cos(var1[4]);
  t703 = Sin(var1[3]);
  t698 = Sin(var1[5]);
  t718 = Cos(var1[3]);
  t754 = -1.*t682;
  t756 = 1. + t754;
  t735 = -1.*t682*t697;
  t739 = -1.*t668*t698;
  t694 = t668*t682;
  t701 = -1.*t697*t698;
  t757 = 0.19*t756;
  t759 = -0.209*t698;
  t761 = 0. + t757 + t759;
  t763 = -0.209*t756;
  t765 = -0.19*t698;
  t766 = 0. + t763 + t765;
  t711 = -1.*t682*t703*t697;
  t715 = -1.*t668*t703*t698;
  t742 = -1.*t668*t682*t703;
  t744 = t703*t697*t698;
  t777 = -0.19*t697;
  t778 = 0. + t777;
  t721 = t718*t682*t697;
  t722 = t718*t668*t698;
  t746 = t718*t668*t682;
  t749 = -1.*t718*t697*t698;

  p_output1(0)=0. + t694 + t701;
  p_output1(1)=0. + t711 + t715;
  p_output1(2)=0. + t721 + t722;
  p_output1(3)=0.;
  p_output1(4)=0.;
  p_output1(5)=0. + t718;
  p_output1(6)=0. + t703;
  p_output1(7)=0.;
  p_output1(8)=0. + t735 + t739;
  p_output1(9)=0. + t742 + t744;
  p_output1(10)=0. + t746 + t749;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.19*(1. - 1.*t668) + 0.19*(t694 + t701) - 0.4165*(t735 + t739) + t668*t761 - 1.*t697*t766;
  p_output1(13)=0. + 0.19*(t711 + t715) + 0.049*(1. - 1.*t718) + 0.111*t718 - 0.4165*(t742 + t744) - 1.*t697*t703*t761 - 1.*t668*t703*t766 - 1.*t703*t778;
  p_output1(14)=0. + 0.062*t703 + 0.19*(t721 + t722) - 0.4165*(t746 + t749) + t697*t718*t761 + t668*t718*t766 + t718*t778;
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_Body_to_FrontLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void H_Body_to_FrontLeftFoot(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



