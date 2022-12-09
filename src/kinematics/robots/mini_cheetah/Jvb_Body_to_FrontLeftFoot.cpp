/*
 * Automatically Generated from Mathematica.
 * Fri 13 Nov 2020 16:04:42 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jvb_Body_to_FrontLeftFoot.h"

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
  double t1935;
  double t1937;
  double t1942;
  double t1944;
  double t1940;
  double t1945;
  double t1948;
  double t1956;
  double t1957;
  double t1951;
  double t1952;
  double t1953;
  double t1959;
  double t1962;
  double t1964;
  double t1970;
  double t1971;
  double t1972;
  double t1973;
  double t1983;
  double t1984;
  double t1985;
  double t1977;
  double t1978;
  double t1979;
  double t1980;
  double t1981;
  double t1982;
  double t1966;
  double t1969;
  double t1974;
  double t1975;
  t1935 = Cos(var1[5]);
  t1937 = Sin(var1[4]);
  t1942 = Cos(var1[4]);
  t1944 = Sin(var1[5]);
  t1940 = -1.*t1935*t1937;
  t1945 = -1.*t1942*t1944;
  t1948 = 0. + t1940 + t1945;
  t1956 = -1.*t1935;
  t1957 = 1. + t1956;
  t1951 = t1935*t1937;
  t1952 = t1942*t1944;
  t1953 = 0. + t1951 + t1952;
  t1959 = -0.209*t1957;
  t1962 = -0.4165*t1935;
  t1964 = 0. + t1959 + t1962;
  t1970 = 0.19*t1957;
  t1971 = 0.19*t1935;
  t1972 = 0.2075*t1944;
  t1973 = 0. + t1970 + t1971 + t1972;
  t1983 = t1942*t1935;
  t1984 = -1.*t1937*t1944;
  t1985 = 0. + t1983 + t1984;
  t1977 = -1.*t1942;
  t1978 = 1. + t1977;
  t1979 = 0.19*t1978;
  t1980 = -1.*t1964*t1937;
  t1981 = t1942*t1973;
  t1982 = 0. + t1979 + t1980 + t1981;
  t1966 = t1942*t1964;
  t1969 = -0.19*t1937;
  t1974 = t1937*t1973;
  t1975 = 0. + t1966 + t1969 + t1974;

  p_output1(0)=0;
  p_output1(1)=0.;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=0.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0.;
  p_output1(8)=0;
  p_output1(9)=0. - 0.111*t1948 - 0.049*t1953;
  p_output1(10)=0. + t1985*(0. - 1.*t1948*t1982 - 1.*t1975*t1985) + t1948*(0. + t1953*t1975 + t1982*t1985);
  p_output1(11)=0. + 0.062*t1985;
  p_output1(12)=0. - 0.19*t1944 - 1.*t1935*t1964 + t1944*t1973;
  p_output1(13)=0.;
  p_output1(14)=0. - 0.19*t1935 + t1944*t1964 + t1935*t1973;
  p_output1(15)=0.2075;
  p_output1(16)=0.;
  p_output1(17)=0.;
  p_output1(18)=0;
  p_output1(19)=0.;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0.;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0.;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0.;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0.;
  p_output1(32)=0;
  p_output1(33)=0;
  p_output1(34)=0.;
  p_output1(35)=0;
}


       
Eigen::Matrix<double,3,12> Jvb_Body_to_FrontLeftFoot(const Eigen::Matrix<double,12,1> &var1)
//void Jvb_Body_to_FrontLeftFoot(Eigen::Matrix<double,3,12> &p_output1, const Eigen::Matrix<double,12,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,12>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



