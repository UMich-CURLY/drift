/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:04:40 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jb_Body_to_FrontLeftFoot.h"

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
static void output1(Eigen::Matrix<double,6,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  double t1893;
  double t1898;
  double t1879;
  double t1899;
  double t1907;
  double t1908;
  double t1910;
  double t1920;
  double t1923;
  double t1914;
  double t1915;
  double t1916;
  double t1925;
  double t1927;
  double t1928;
  double t1931;
  double t1932;
  double t1933;
  double t1934;
  double t1897;
  double t1901;
  double t1902;
  double t1938;
  double t1939;
  double t1940;
  double t1941;
  double t1942;
  double t1943;
  double t1929;
  double t1930;
  double t1935;
  double t1936;
  t1893 = Cos(var1[5]);
  t1898 = Sin(var1[4]);
  t1879 = Cos(var1[4]);
  t1899 = Sin(var1[5]);
  t1907 = -1.*t1893*t1898;
  t1908 = -1.*t1879*t1899;
  t1910 = 0. + t1907 + t1908;
  t1920 = -1.*t1893;
  t1923 = 1. + t1920;
  t1914 = t1893*t1898;
  t1915 = t1879*t1899;
  t1916 = 0. + t1914 + t1915;
  t1925 = -0.209*t1923;
  t1927 = -0.4165*t1893;
  t1928 = 0. + t1925 + t1927;
  t1931 = 0.19*t1923;
  t1932 = 0.19*t1893;
  t1933 = 0.2075*t1899;
  t1934 = 0. + t1931 + t1932 + t1933;
  t1897 = t1879*t1893;
  t1901 = -1.*t1898*t1899;
  t1902 = 0. + t1897 + t1901;
  t1938 = -1.*t1879;
  t1939 = 1. + t1938;
  t1940 = 0.19*t1939;
  t1941 = -1.*t1928*t1898;
  t1942 = t1879*t1934;
  t1943 = 0. + t1940 + t1941 + t1942;
  t1929 = t1879*t1928;
  t1930 = -0.19*t1898;
  t1935 = t1898*t1934;
  t1936 = 0. + t1929 + t1930 + t1935;

  p_output1(0)=0;
  p_output1(1)=0.;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=0.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0.;
  p_output1(8)=0;
  p_output1(9)=0;
  p_output1(10)=0.;
  p_output1(11)=0;
  p_output1(12)=0;
  p_output1(13)=0.;
  p_output1(14)=0;
  p_output1(15)=0;
  p_output1(16)=0.;
  p_output1(17)=0;
  p_output1(18)=t1902;
  p_output1(19)=0.;
  p_output1(20)=t1910;
  p_output1(21)=0. - 0.111*t1910 - 0.049*t1916;
  p_output1(22)=0. + t1910*(0. + t1916*t1936 + t1902*t1943) + t1902*(0. - 1.*t1902*t1936 - 1.*t1910*t1943);
  p_output1(23)=0. + 0.062*t1902;
  p_output1(24)=0.;
  p_output1(25)=-1.;
  p_output1(26)=0.;
  p_output1(27)=0. - 0.19*t1899 - 1.*t1893*t1928 + t1899*t1934;
  p_output1(28)=0.;
  p_output1(29)=0. - 0.19*t1893 + t1899*t1928 + t1893*t1934;
  p_output1(30)=0.;
  p_output1(31)=-1.;
  p_output1(32)=0.;
  p_output1(33)=0.2075;
  p_output1(34)=0.;
  p_output1(35)=0.;
  p_output1(36)=0;
  p_output1(37)=0.;
  p_output1(38)=0;
  p_output1(39)=0;
  p_output1(40)=0.;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0.;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0.;
  p_output1(47)=0;
  p_output1(48)=0;
  p_output1(49)=0.;
  p_output1(50)=0;
  p_output1(51)=0;
  p_output1(52)=0.;
  p_output1(53)=0;
  p_output1(54)=0;
  p_output1(55)=0.;
  p_output1(56)=0;
  p_output1(57)=0;
  p_output1(58)=0.;
  p_output1(59)=0;
  p_output1(60)=0;
  p_output1(61)=0.;
  p_output1(62)=0;
  p_output1(63)=0;
  p_output1(64)=0.;
  p_output1(65)=0;
  p_output1(66)=0;
  p_output1(67)=0.;
  p_output1(68)=0;
  p_output1(69)=0;
  p_output1(70)=0.;
  p_output1(71)=0;
}


       
Eigen::Matrix<double,6,12> Jb_Body_to_FrontLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jb_Body_to_FrontLeftFoot(Eigen::Matrix<double,6,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,6,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



