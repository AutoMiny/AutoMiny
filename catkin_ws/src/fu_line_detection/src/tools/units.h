#ifndef UNITS_H_
#define UNITS_H_

/** @ingroup units
 ** @{
 */

#include <boost/units/unit.hpp>
#include <boost/units/quantity.hpp>

#include <boost/units/cmath.hpp>

#include <boost/units/systems/si.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <boost/units/systems/si/io.hpp>

#include <boost/units/make_scaled_unit.hpp>

#include "has_member.h"


/*------------------------------------------------------------------------------------------------*/
/* UTILITIES */
/*------------------------------------------------------------------------------------------------*/

/** @brief Get the unit type as a string.
 **
 ** Example:
 ** @code
 ** Degree angle;
 ** std::cout << "Angle: " << angle.value() << " " << unit2str(angle) << std::endl;
 ** @endcode
 **
 ** @return the name of the unit as string
 */
template <typename Unit, typename T>
std::string unit2str(const boost::units::quantity<Unit, T> &) {
	return boost::units::symbol_string(Unit());
}

#define UNIT2STRING(value) (unit2str(value).c_str())


/*------------------------------------------------------------------------------------------------*/
/* TIME */
/*------------------------------------------------------------------------------------------------*/

typedef boost::units::si::time second;
using boost::units::si::seconds;

typedef boost::units::make_scaled_unit<
		second, boost::units::scale<60, boost::units::static_rational<1> >
	>::type minute;
BOOST_UNITS_STATIC_CONSTANT(minutes, minute);

typedef boost::units::make_scaled_unit<
		second, boost::units::scale<10, boost::units::static_rational<-3> >
	>::type millisecond;
BOOST_UNITS_STATIC_CONSTANT(milliseconds, millisecond);

typedef boost::units::make_scaled_unit<
		second, boost::units::scale<10, boost::units::static_rational<-6> >
	>::type microsecond;
BOOST_UNITS_STATIC_CONSTANT(microseconds, microsecond);

typedef boost::units::make_scaled_unit<
		second, boost::units::scale<10, boost::units::static_rational<-9> >
	>::type nanosecond;
BOOST_UNITS_STATIC_CONSTANT(nanoseconds, nanosecond);

typedef boost::units::quantity<minute,      double> Minute;
typedef boost::units::quantity<second,      double> Second;
typedef boost::units::quantity<millisecond, double> Millisecond;
typedef boost::units::quantity<microsecond, double> Microsecond;
typedef boost::units::quantity<nanosecond,  double> Nanosecond;

typedef boost::units::quantity<millisecond, uint64_t> Timestamp;
typedef boost::units::quantity<microsecond, uint64_t> MicroTimestamp;
typedef boost::units::quantity<nanosecond,  uint64_t> NanoTimestamp;


/*------------------------------------------------------------------------------------------------*/
/* FREQUENCY */
/*------------------------------------------------------------------------------------------------*/

#include <boost/units/systems/si/frequency.hpp>
using boost::units::si::hertz;
using boost::units::si::frequency;

// hertz, e.g. frames per second
typedef boost::units::quantity<frequency, double> Hertz;

// boost does not convert types, so in order to support 1/Hz (in addition to 1.0/Hz), add this operator
inline Hertz operator/(const int &count, const Second &sec)  { return double(count) / sec; }

// rounds per minute
typedef boost::units::make_scaled_unit<
		frequency, boost::units::scale<60, boost::units::static_rational<-1> >
	>::type rpm_type;
BOOST_UNITS_STATIC_CONSTANT(rounds_per_minute, rpm_type);
typedef boost::units::quantity<rpm_type, double> RPM;

// boost does not convert types, so in order to support 1/RPM (in addition to 1.0/RPM), add this operator
inline RPM operator/(const int &count, const Minute &min)  { return double(count) / min; }



/*------------------------------------------------------------------------------------------------*/
/* LENGTH */
/*------------------------------------------------------------------------------------------------*/

typedef boost::units::si::length meter;
using boost::units::si::meters;

typedef boost::units::make_scaled_unit<
		meter, boost::units::scale<10, boost::units::static_rational<-3> >
	>::type millimeter;
BOOST_UNITS_STATIC_CONSTANT(millimeters, millimeter);

typedef boost::units::make_scaled_unit<
		meter, boost::units::scale<10, boost::units::static_rational<-2> >
	>::type centimeter;
BOOST_UNITS_STATIC_CONSTANT(centimeters, centimeter);

// granularity is never finer than millimeter
typedef boost::units::quantity<meter,      double> Meter;
typedef boost::units::quantity<centimeter, double> Centimeter;
typedef boost::units::quantity<millimeter, double> Millimeter;

// all possible combinations for addition (convert to smaller unit)
inline Millimeter operator+(const Meter &m, const Millimeter &mm)        { return Millimeter(m)  + mm; }
inline Millimeter operator+(const Millimeter &mm, const Meter &m)        { return Millimeter(m)  + mm; }
inline Centimeter operator+(const Meter &m, const Centimeter &cm)        { return Centimeter(m)  + cm; }
inline Centimeter operator+(const Centimeter &cm, const Meter &m)        { return Centimeter(m)  + cm; }
inline Millimeter operator+(const Centimeter &cm, const Millimeter &mm)  { return Millimeter(cm) + mm; }
inline Millimeter operator+(const Millimeter &mm, const Centimeter &cm)  { return Millimeter(cm) + mm; }

/*------------------------------------------------------------------------------------------------*/
/* INVERSE LENGTH */
/*------------------------------------------------------------------------------------------------*/

#include <boost/units/systems/si/wavenumber.hpp>
typedef boost::units::si::wavenumber reciprocal_meter;
using boost::units::si::reciprocal_meters;

typedef boost::units::make_scaled_unit<
               reciprocal_meter, boost::units::scale<10, boost::units::static_rational<2> >
       >::type reciprocal_centimeter;
BOOST_UNITS_STATIC_CONSTANT(reciprocal_centimeters, reciprocal_centimeter);

typedef boost::units::make_scaled_unit<
               reciprocal_meter, boost::units::scale<10, boost::units::static_rational<3> >
       >::type reciprocal_millimeter;
BOOST_UNITS_STATIC_CONSTANT(reciprocal_millimeters, reciprocal_millimeter);

typedef boost::units::quantity<reciprocal_meter,      double> PerMeter;
typedef boost::units::quantity<reciprocal_centimeter, double> PerCentimeter;
typedef boost::units::quantity<reciprocal_millimeter, double> PerMillimeter;

/*------------------------------------------------------------------------------------------------*/
/* VELOCITIES */
/*------------------------------------------------------------------------------------------------*/

#include <boost/units/systems/si/velocity.hpp>

typedef boost::units::si::velocity mps;
using boost::units::si::meters_per_second;
typedef boost::units::quantity<mps> MPS;


/*------------------------------------------------------------------------------------------------*/
/* ACCELERATION */
/*------------------------------------------------------------------------------------------------*/

#include <boost/units/systems/si/acceleration.hpp>

typedef boost::units::si::acceleration mpss;
using boost::units::si::meters_per_second_squared;
typedef boost::units::quantity<mpss> MPSS;


/*------------------------------------------------------------------------------------------------*/
/* ANGLES */
/*------------------------------------------------------------------------------------------------*/

typedef boost::units::si::plane_angle radian;
using boost::units::si::radians;

#include <boost/units/systems/angle/degrees.hpp>
typedef boost::units::degree::plane_angle degree;
using boost::units::degree::degrees;

typedef boost::units::quantity<radian> Radian;
typedef boost::units::quantity<degree> Degree;

// all possible combinations for addition (convert to radian)
inline Radian operator+(const Radian &rad, const Degree &deg)  { return Radian(deg) + rad; }
inline Radian operator+(const Degree &deg, const Radian &rad)  { return Radian(deg) + rad; }


/*------------------------------------------------------------------------------------------------*/
/* ANGULAR VELOCITIES */
/*------------------------------------------------------------------------------------------------*/

#include <boost/units/systems/si/angular_velocity.hpp>

typedef boost::units::si::angular_velocity rps; // radian per second
using boost::units::si::radians_per_second;
typedef boost::units::quantity<rps> RPS;

typedef boost::units::divide_typeof_helper<boost::units::degree::plane_angle, boost::units::si::time>::type dps;
BOOST_UNITS_STATIC_CONSTANT(degrees_per_second, dps);
typedef boost::units::quantity<dps> DPS;


/*------------------------------------------------------------------------------------------------*/
/* Temperature */
/*------------------------------------------------------------------------------------------------*/

#include <boost/units/systems/temperature/celsius.hpp>

typedef boost::units::quantity<boost::units::celsius::temperature> Celsius;
BOOST_UNITS_STATIC_CONSTANT(celsius, boost::units::celsius::temperature);


/*------------------------------------------------------------------------------------------------*/
/* Electric Potential */
/*------------------------------------------------------------------------------------------------*/

#include <boost/units/systems/si/electric_potential.hpp>

typedef boost::units::quantity<boost::units::si::electric_potential> Volt;
BOOST_UNITS_STATIC_CONSTANT(volts, boost::units::si::electric_potential);


/*------------------------------------------------------------------------------------------------*/
/* Operator templates for all arithmetic types */
/*------------------------------------------------------------------------------------------------*/

/// unit * scalar
template< class UNIT, class Y, class SCALAR
        , class = typename std::enable_if< std::is_arithmetic< SCALAR >::value>::type
        >
inline boost::units::quantity<UNIT, Y> operator*(const boost::units::quantity<UNIT, Y> &value, const SCALAR &scalar) {
	return static_cast<Y>(scalar) * value;
}

/// scalar * unit (for some types of scalar already provided)
template< class UNIT, class Y, class SCALAR
        , class = typename std::enable_if< std::is_arithmetic< SCALAR >::value>::type
        >
inline boost::units::quantity<UNIT, Y> operator*(const SCALAR &scalar, const boost::units::quantity<UNIT, Y> &value) {
	return static_cast<Y>(scalar) * value;
}

/// unit / scalar
template< class UNIT, class Y, class SCALAR
        , class = typename std::enable_if< std::is_arithmetic< SCALAR >::value>::type
        >
inline boost::units::quantity<UNIT, Y> operator/(const boost::units::quantity<UNIT, Y> &value, const SCALAR &scalar) {
	return boost::units::quantity<UNIT, Y>::from_value(value.value() / static_cast<Y>(scalar));
}

/*------------------------------------------------------------------------------------------------*/
/* check whether a type is a unit */
/*------------------------------------------------------------------------------------------------*/

struct check_is_unit {
	template<typename T,
	         const typename T::value_type& (T::*)() const = &T::value
	        >
	struct get {
	};
};

template<typename T>
struct is_unit: has_member<T, check_is_unit> {
};

/** @} */

#endif /* UNITS_H_ */
