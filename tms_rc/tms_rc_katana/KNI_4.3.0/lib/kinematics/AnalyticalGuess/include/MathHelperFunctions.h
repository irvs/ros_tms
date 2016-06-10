/***************************************************************************
 *   Copyright (C) 2006-2008 by Neuronics AG                               *
 *   support@neuronics.ch                                                  *
 ***************************************************************************/

#define  MHF_PI      3.14159265358979323846

#include <cmath>
#include <vector>
#include <functional>
#include "exception.h"

#ifndef MATH_HELPER_FUNCTIONS
#define MATH_HELPER_FUNCTIONS



namespace MHF {

#ifdef WIN32
template<typename _T> inline double round(_T x)
// Copyright (C) 2001 Tor M. Aamodt, University of Toronto
// Permisssion to use for all purposes commercial and otherwise granted.
// THIS MATERIAL IS PROVIDED "AS IS" WITHOUT WARRANTY, OR ANY CONDITION OR
// OTHER TERM OF ANY KIND INCLUDING, WITHOUT LIMITATION, ANY WARRANTY
// OF MERCHANTABILITY, SATISFACTORY QUALITY, OR FITNESS FOR A PARTICULAR
// PURPOSE.
{
   if( x > 0 ) {
       __int64 xint = (__int64) (x+0.5);
       if( xint % 2 ) {
           // then we might have an even number...
           double diff = x - (double)xint;
           if( diff == -0.5 )
               return double(xint-1);
       }
       return double(xint);
   } else {
       __int64 xint = (__int64) (x-0.5);
       if( xint % 2 ) {
           // then we might have an even number...
           double diff = x - (double)xint;
           if( diff == 0.5 )
               return double(xint+1);
       }
       return double(xint);
   }
}
#endif // ifdef WIN32
//*****************************************************************

template<typename _T> inline short sign(_T x) { return ( (x<0) ? -1 : 1 ); }

//*****************************************************************

///
/// function-object which calculates sinus for n-elements of a container if used together
/// with a STL algorithm
template<typename _T> struct unary_precalc_sin : public std::unary_function<_T, _T> {
    _T operator() (_T &x) {
	return sin(x);
    }
};

///
/// \sa unary_precalc_sin
///
template<typename _T> struct unary_precalc_cos : public std::unary_function<_T, _T> {
    _T operator() (_T x) {
	return cos(x);
    }
};



//*****************************************************************
template<typename _T> inline _T atan1(_T in1, _T in2) {

    if(in1==0.0)
	return MHF_PI+sign(in2)*MHF_PI/2;

    if(in1<0.0)
	return atan(in2/in1)+MHF_PI;

    if( (in1>0.0) && (in2<0.0) )
	return atan(in2/in1)+2.0*MHF_PI;

    return atan(in2/in1);
}

//*****************************************************************
template<typename _T> inline _T acotan(const _T in) {
    if(in == 0.0)
	return MHF_PI/2;
    else
	return atan(1/in);
}

//*************************************************
template<typename _T> inline _T atan0(const _T in1, const _T in2) {
    if(in1 == 0.0)
	return MHF_PI/2;
    return atan(in2/in1);
}

//*************************************************
template<typename _T> inline _T pow2(const _T in) {
  return pow(in,2);
}


///
/// conversion from radian to degree
///
template<typename _T> inline _T rad2deg(const _T a) {
    return a*(180.0/MHF_PI);
}

///
/// a function-object version of rad2deg
///
template<typename _T> struct unary_rad2deg : public std::unary_function<_T, _T> {
    _T operator() (const _T a) { return rad2deg(a); }
};

///
/// conversion from degree to radian
///
template<typename _T> inline _T deg2rad(const _T a) {
    return a*(MHF_PI/180.0);
}

///
/// a function-object version of rad2deg
///
template<typename _T> struct unary_deg2rad : public std::unary_function<_T, _T> {
  _T operator() (const _T a) { deg2rad(a); }
};

//*************************************************
template<typename _T> _T inline anglereduce(const _T a) {
    return a - floor( a/(2*MHF_PI) )*2*MHF_PI;
}
//*************************************************

///
/// converts absolute angles in radian to encoders.
///
template<typename _angleT, typename _encT> inline _encT rad2enc(_angleT const& angle, _angleT const& angleOffset, _encT const& epc, _encT const& encOffset, _encT const& rotDir) {
    // converting all parameters to _angleT (usually =double)
    _angleT _epc = epc, _rotDir = rotDir, _angleOffset = angleOffset, _encOffset = encOffset;
    return static_cast<_encT>( round( _encOffset + (_angleOffset-angle)*_epc*_rotDir/(2*MHF_PI) ) );
}

///
/// converts encoders to absolute angles in radian
///
template<typename _angleT, typename _encT> inline _angleT enc2rad(_encT const& enc, _angleT const& angleOffset, _encT const& epc, _encT const& encOffset, _encT const& rotDir) {
    // converting all parameters to _angleT (usually = double)
    _angleT _epc = epc, _rotDir = rotDir, _angleOffset = angleOffset, _encOffset = encOffset, _enc = enc;
    return _angleOffset -  (_enc - _encOffset)*2.0*MHF_PI/(_epc*_rotDir);
}

///
/// Find the first equal angle.
/// You have to pass a cos and a sin Value
inline double findFirstEqualAngle(double cosValue, double sinValue, double tolerance) {
    double v1[2], v2[2];

		v1[0] = acos(cosValue);
		v1[1] = -v1[0];
		v2[0] = asin(sinValue);
		v2[1] = MHF_PI - v2[0];

    for(int i=0;i<2;++i) {
		  for(int j=0;j<2;++j) {
			if(std::abs(anglereduce(v1[i]) - anglereduce(v2[j])) < tolerance) return v1[i];
		  }
    }
	throw AnaGuess::Exception("precondition for findFirstEqualAngle failed -> no equal angles found", -2);
    return 0;
}


} // namespace



#endif // ifndef MATH_HELPER_FUNCTIONS
