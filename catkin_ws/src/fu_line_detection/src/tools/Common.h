#ifndef __Math_Common_h__
#define __Math_Common_h__

#include <math.h>
#include <stdlib.h>
#include <ctime>
#include <cmath>

#include "attributes.h"
#include "units.h"


namespace Math {

/** constant for a half circle*/
const double pi = 3.1415926535897932384626433832795;

/** constant for a full circle*/
const double pi2 = 2.0 * 3.1415926535897932384626433832795;

/** constant for three quarter circle*/
const double pi3_2 = 1.5 * 3.1415926535897932384626433832795;

/** constant for a quarter circle*/
const double pi_2 = 0.5 * 3.1415926535897932384626433832795;

/** constant for 1 degree in radian*/
const double pi_180 = 3.1415926535897932384626433832795 / 180.0;

/** constant for a 1/8 circle*/
const double pi_4 = 3.1415926535897932384626433832795 * 0.25;

/** constant for a 3/4 circle*/
const double pi3_4 = 3.1415926535897932384626433832795 * 0.75;

/** constant for an expression used by the gaussian function*/
const double sqrt2pi = sqrt(2.0 * pi);

/** constant for e*/
const double e = 2.71828182845902353602874713527;

/** the gravity of degree of latitude 45 */
const double g = 9.80620;

/**
 * defines the isNan function for linux and windows
 */
template<class T>
bool isNan(T f) {
#ifdef WIN32
	return _isnan(static_cast<double> (f)) != 0;
#else
	return std::isnan(f);
#endif
}

/**
 * defines the isInf function for linux and windows
 */
inline bool isInf(double x) {
#ifdef WIN32
	return !_finite(x);
#else
	return std::isinf(x);
#endif
}

/**
 * defines the sign of a value
 */
#ifdef sgn
#undef sgn
#endif

template<class T>
int sgn(const T& value) {
	return (value < 0 ? -1 : (0 == value ? 0 : 1));
}

/**
 * defines the square of a value
 */
#ifdef sqr
#undef sqr
#endif

template<class T>
T sqr(const T& value) {
	return value * value;
}

/**
 * mathematical rounding for floating point types
 */
template<class T>
T round(const T& value) {
	return floor(value + static_cast<T>(0.5));
}

inline double sec(const double value) {
	return 1.0 / cos(value);
}
inline double cosec(const double value) {
	return 1.0 / sin(value);
}

/** @deprecated
 * Converts angle from rad to degrees.
 * This function is deprecated, use the unit types instead.
 *
 * @param angle code in rad
 * @return angle coded in degrees
 */

DEPRECATED inline double toDegrees(double angle) {
	return angle * 180.0 / pi;
}

/** @deprecated
 * Converts angle from degrees to rad.
 * This function is deprecated, use the unit types instead.
 *
 * @param degrees angle coded in degrees
 * @return angle coded in rad
 */

DEPRECATED inline double fromDegrees(double degrees) {
	return degrees * pi_180;
}


/*------------------------------------------------------------------------------------------------*/

/**
 ** Reduce angle to [-pi..+pi[
 **
 ** @param angle angle to normalize
 ** @return normalized angle
 */

inline Radian normalize(Radian angle) {
	if (angle < pi * radians && angle >= -pi * radians)
		return angle;

	/* from -2pi to 2*pi */
	double normalizedAngle = fmod(angle.value(), pi2);

	/* from -pi to pi */
	if (normalizedAngle >= pi) {
		normalizedAngle = normalizedAngle - pi2;
	} else if (normalizedAngle < -pi)
		normalizedAngle = normalizedAngle + pi2;

	return normalizedAngle * radians;
}


/*------------------------------------------------------------------------------------------------*/

/** Reduce angle to [0..pi[.
 **
 ** @param angle angle to reduce
 ** @return normalized angle in [0..pi[
 */

inline Radian normalizePositive(Radian angle) {
	Radian normalizedAngle = normalize(angle);
	if (normalizedAngle < 0*radians) {
		normalizedAngle += pi*radians;
	}

	return normalizedAngle;
}

/** Reduce angle to [0..2pi[.
 **
 ** @param angle angle to reduce
 ** @return normalized angle in [0..2pi[
 */

inline Radian normalizePositive2PI(Radian angle) {
	return fmod(fmod(angle.value(), pi2) + pi2, pi2) * radians;
}



/*------------------------------------------------------------------------------------------------*/

/**
 ** Reduce angle to [-180..+180[
 **
 ** @param angle angle to normalize
 ** @return normalized angle
 */

inline Degree normalize(Degree angle) {
	if (angle < 180 * degrees && angle >= -180 * degrees)
		return angle;

	/* from -360 to 360 */
	double normalizedAngle = fmod(angle.value(), 360.);

	/* from -180 to 180 */
	if (normalizedAngle >= 180.) {
		normalizedAngle = normalizedAngle - 360.;
	} else if (normalizedAngle < -180) {
		normalizedAngle = normalizedAngle + 360.;
	}

	return normalizedAngle * degrees;
}


/*------------------------------------------------------------------------------------------------*/

/** Reduce angle to [0..180[.
 **
 ** @param angle angle to reduce
 ** @return normalized angle in [0..180[
 */

inline Degree normalizePositive(Degree angle) {
	Degree normalizedAngle = normalize(angle);
	if (normalizedAngle < 0*degrees) {
		normalizedAngle += 180*degrees;
	}

	return normalizedAngle;
}

/** Reduce angle to [0..360[.
 **
 ** @param angle angle to reduce
 ** @return normalized angle in [0..360[
 */

inline Degree normalizePositive360(Degree angle) {
	return fmod(fmod(angle.value(), 360.) + 360, 360) * degrees;
}


/**
 * test if a given angle is between two bounds (counterclock-like described)
 * @param lowerBound
 * @param upperBound
 * @param angle
 * @return
 */


inline bool isBetween(Degree lowerBound, Degree upperBound, Degree angle) {
	double low  = Math::normalizePositive360(angle - lowerBound).value();
	double high = Math::normalizePositive360(upperBound - lowerBound).value();
	return (high >= low);
}

/**
 * test if a given angle is between two bounds (counterclock-like described)
 * @param lowerBound
 * @param upperBound
 * @param angle
 * @return
 */
inline bool isBetween(Radian lowerBound, Radian upperBound, Radian angle) {
	double low  = Math::normalizePositive2PI(angle - lowerBound).value();
	double high = Math::normalizePositive2PI(upperBound - lowerBound).value();
	return (high >= low);
}

inline Degree normalize90To90(Degree angle) {
	if (angle > 90*degrees || angle < -90*degrees)
		return Math::normalize(angle+180*degrees);
	else
		return angle;
}

/*------------------------------------------------------------------------------------------------*/

/** Determine unsigned delta (i.e. in [0,180[) between two angles
 **
 ** @param angle1 first angle
 ** @param angle2 second angle
 ** @return delta of angles (angle1 to angle2) in [0..180[
 */

inline Degree angleUnsignedDelta(Degree angle1, Degree angle2) {
	return abs( normalize(angle2 - angle1) );
}


/*------------------------------------------------------------------------------------------------*/

/** Determine unsigned delta (i.e. in [0,pi[) between two angles
 **
 ** @param angle1 first angle
 ** @param angle2 second angle
 ** @return delta of angles (angle1 to angle2) in [0..pi[
 */

inline Radian angleUnsignedDelta(Radian angle1, Radian angle2) {
	return abs( normalize(angle2 - angle1) );
}



/*------------------------------------------------------------------------------------------------*/

/** Determine signed delta (i.e. in [-180,180[) between two angles
 **
 ** @param angle1 first angle
 ** @param angle2 second angle
 ** @return delta of angles (angle1 to angle2) in [-180..180[, i.e. the angle
 **         with the smallest absolute value to add to angle1 to get angle2
 **
 ** Example: angleSignedDelta(10, -10) => -20
 **          angleSignedDelta(-10, 10) => 20
 */

inline Degree angleSignedDelta(Degree angle1, Degree angle2) {
	return normalize(angle2 - angle1);
}


/*------------------------------------------------------------------------------------------------*/

/** Determine signed delta (i.e. in [-pi,pi[) between two angles
 **
 ** @param angle1 first angle
 ** @param angle2 second angle
 ** @return delta of angles (angle1 to angle2) in [-pi..pi[, i.e. the angle
 **         with the smallest absolute value to add to angle1 to get angle2
 */

inline Radian angleSignedDelta(Radian angle1, Radian angle2) {
	return normalize(angle2 - angle1);
}


/*------------------------------------------------------------------------------------------------*/

/**
 * The function returns a random number in the range of [0..1].
 * @return The random number.
 */
inline double random() {
	return double(rand()) / RAND_MAX;
}

/**
 * The function returns a random integer number in the range of [0..n-1].
 * @param n the number of possible return values (0 ... n-1)
 * @return The random number.
 */
inline int random(int n) {
	return (int) (random() * n * 0.999999);
}


/*------------------------------------------------------------------------------------------------*/

/**
 * set the value to range [min,max]
 * @param[in] x the orginal value
 * @param[in] min the range left
 * @param[in] max the range right
 * @return the value be clamped
 */

template<class T>
T clamp(T x, T min, T max) {
	if (x > max)
		return max;
	if (x < min)
		return min;
	return x;
}


/*------------------------------------------------------------------------------------------------*/

/**
 * draw number from normal random distribution
 * @param mean the mean value
 * @param std the standard deviation
 * @return the random number
 */
inline double normal(const double &mean, const double &std) {
	static const double r_max = RAND_MAX + 1.0;
	return std * sqrt(-2.0 * log((rand() + 1.0) / r_max))
	           * sin(2.0 * pi * rand() / r_max) + mean;
}


/*------------------------------------------------------------------------------------------------*/

/** This method returns the bisector (average) of two angles. It deals
 **  with the boundary problem, thus when angleMin equals 170 and angleMax
 **  equals -100, -145 is returned.
 **  @param[in] angleMin minimum angle [-pi,+pi]
 **  @param[in] angleMax maximum angle [-pi,+pi]
 **  @return average of angleMin and angleMax.
 */

inline Radian calculateMeanAngle(Radian angleMin, Radian angleMax) {
	angleMin = normalize(angleMin);
	angleMax = normalize(angleMax);
	Radian angle = (angleMin + angleMax) * 0.5;
	if (angleMin > angleMax)
		angle += pi*radians;
	return normalize(angle);
}

/*------------------------------------------------------------------------------------------------*/

/** This method returns the bisector (average) of two angles. It deals
 **  with the boundary problem, thus when angleMin equals 170 and angleMax
 **  equals -100, -145 is returned.
 **  @param[in] angleMin minimum angle [-180,+180]
 **  @param[in] angleMax maximum angle [-180,+180]
 **  @return average of angleMin and angleMax.
 */

inline Degree calculateMeanAngle(Degree angleMin, Degree angleMax) {
	angleMin = normalize(angleMin);
	angleMax = normalize(angleMax);
	Degree angle = (angleMin + angleMax) * 0.5;
	if (angleMin > angleMax)
		angle += 180*degrees;
	return normalize(angle);
}

}  //end namespace Math

#endif // __Math_Common_h__
