#ifndef VECTOR_H_
#define VECTOR_H_

#include <iostream>
#include <string>
#include <type_traits>
#include <boost/units/cmath.hpp>

#include "units.h"
#include "Common.h"

/**
 * This is a 2-dimensional vector template that can be used with (signed) integer and
 * floating point types as well as with the boost quantities used in this project.
 *
 * MemberType is the type of the contained values.
 */
template <typename MemberType>
class FuVector {
	/**
	 * This contains specialized behavior for MemberType. It will be privately
	 * specialized later in this class for all supported types. These are all
	 * signed arithmetic types as well as the boost::units::quantities used in
	 * this project, which are detected by the existence of their value()
	 * member.
	 *
	 * Unsupported MemberTypes will fail the following static assertion.
	 */
	template <typename Type, typename Enable = void>
	struct MemberTypeWrapperFor {
		typedef void type;
	};

	static_assert (
		not std::is_void<typename MemberTypeWrapperFor<MemberType>::type>::value,
		"MemberType must either be a signed arithmetic type or a boost::units::quantity."
	);

public:
	typedef MemberTypeWrapperFor<MemberType> MemberTypeWrapper;
	typedef typename MemberTypeWrapper::ScalarType ScalarType;
	typedef typename MemberTypeWrapper::RealDomainType RealDomainMemberType;

	FuVector() : FuVector {MemberType(), MemberType()} { /* void */ }
	FuVector(MemberType x, MemberType y) : x(x), y(y) { /* void */ }

	MemberType getX() const { return x; }
	MemberType getY() const { return y; }

	std::string toString() const {
		std::stringstream result;
		result << "FuVector {" << getX() << ", " << getY() << "}";
		return result.str();
	}

	RealDomainMemberType magnitude() const {
		return sqrt(squaredMagnitude());
	}

	decltype(MemberType() * MemberType()) squaredMagnitude() const {
		return (*this) * (*this);
	}

	Radian direction() const {
		return std::atan2(
			MemberTypeWrapper::toScalar(this->y),
			MemberTypeWrapper::toScalar(this->x)
		) * radians;
	}

	template <typename OtherMemberType>
	Radian angleTo(const FuVector<OtherMemberType>& other) const {
		return Math::normalize(other.direction() - this->direction());
	}

	FuVector<MemberType> operator-() const {
		return FuVector<MemberType>{-x, -y};
	}

	FuVector<MemberType> operator+(MemberType value) const {
		return FuVector<MemberType>(*this) += value;
	}

	FuVector<MemberType> operator-(MemberType value) const {
		return FuVector<MemberType>(*this) -= value;
	}

	FuVector<MemberType> operator*(ScalarType scalar) const {
		return FuVector<MemberType>(*this) *= scalar;
	}

	FuVector<MemberType> operator/(ScalarType scalar) const {
		return FuVector<MemberType>(*this) /= scalar;
	}

	FuVector<MemberType> operator+(const FuVector<MemberType>& other) const {
		return FuVector<MemberType>(*this) += other;
	}

	FuVector<MemberType> operator-(const FuVector<MemberType>& other) const {
		return FuVector<MemberType>(*this) -= other;
	}

	template <typename OtherMemberType>
	decltype(MemberType() * OtherMemberType()) operator*(
		const FuVector<OtherMemberType>& other
	) const {
		return this->x * other.getX() + this->y * other.getY();
	}

	bool operator==(const FuVector<MemberType>& other) const {
		return this->x == other.x and this->y == other.y;
	}

	bool operator!=(const FuVector<MemberType>& other) const {
		return !(*this == other);
	}

	FuVector<MemberType> rotatedBy(Radian angle) const {
		return FuVector<MemberType>(*this).rotate(angle);
	}

	FuVector<MemberType>& operator+=(MemberType value) {
		this->x += value;
		this->y += value;
		return *this;
	}

	FuVector<MemberType>& operator-=(MemberType value) {
		this->x -= value;
		this->y -= value;
		return *this;
	}

	FuVector<MemberType>& operator*=(ScalarType scalar) {
		this->x *= scalar;
		this->y *= scalar;
		return *this;
	}

	FuVector<MemberType>& operator/=(ScalarType scalar) {
		this->x /= scalar;
		this->y /= scalar;
		return *this;
	}

	FuVector<MemberType>& operator+=(const FuVector<MemberType>& other) {
		this->x += other.x;
		this->y += other.y;
		return *this;
	}

	FuVector<MemberType>& operator-=(const FuVector<MemberType>& other) {
		this->x -= other.x;
		this->y -= other.y;
		return *this;
	}

	FuVector<MemberType>& rotate(Radian angle) {
		double sin = boost::units::sin(angle);
		double cos = boost::units::cos(angle);
		MemberType x = this->x;
		MemberType y = this->y;
		this->x = x * cos - y * sin;
		this->y = x * sin + y * cos;
		return *this;
	}

	/**
	 * Serialize vector to stream, dropping all units.
	 */
	friend std::ostream& operator<<(std::ostream& stream, const FuVector<MemberType>& vector) {
		stream <<
			"(" <<
			MemberTypeWrapper::toScalar(vector.x) <<
			" " <<
			MemberTypeWrapper::toScalar(vector.y) <<
			")";
		return stream;
	}

	/**
	 * Read vector from stream, assuming MemberType as units.
	 */
	friend std::istream& operator>>(std::istream& stream, FuVector<MemberType>& vector) {
		std::istream::sentry sentry(stream, true);
		if (!sentry) return stream;

		stream.setf(std::ios_base::skipws);
		stream.ignore(std::numeric_limits<std::streamsize>::max(), '(');

		// Read whitespace separated values;
		typename FuVector<MemberType>::ScalarType x, y;
		stream >> x >> y;

		stream.ignore(std::numeric_limits<std::streamsize>::max(), ')');

		// Only save read data if we were successful. Attach the base unit of
		// MemberType using from_value if it is a boost unit, or alternatively
		// hand the value through in case MemberType is a floating point type.
		vector.x = MemberTypeWrapper::fromScalar(x);
		vector.y = MemberTypeWrapper::fromScalar(y);

		return stream;
	}

private:
	/**
	 * Specialized behavior for floating point MemberTypes.
	 */
	template <typename Type>
	struct MemberTypeWrapperFor<
		Type,
		typename std::enable_if<
			std::is_floating_point<Type>::value and
			std::is_signed<Type>::value
		>::type
	> {
		typedef Type type;
		typedef Type ScalarType;
		typedef Type RealDomainType;

		static type fromScalar(ScalarType scalar) { return scalar; }
		static ScalarType toScalar(type value) { return value; }
	};

	/**
	 * Specialized behavior for integer MemberTypes.
	 */
	template <typename Type>
	struct MemberTypeWrapperFor<
		Type,
		typename std::enable_if<
			std::is_integral<Type>::value and
			std::is_signed<Type>::value
		>::type
	> {
		typedef Type type;
		typedef Type ScalarType;
		typedef double RealDomainType;

		static type fromScalar(ScalarType scalar) { return scalar; }
		static ScalarType toScalar(type value) { return value; }
	};

	/**
	 * Specialized behavior for boost unit MemberTypes (detected by their value() member).
	 */
	template <typename Type>
	struct MemberTypeWrapperFor<
		Type,
		typename std::enable_if<
			std::is_member_function_pointer<decltype(&Type::value)>::value
		>::type
	> {
		typedef Type type;
		typedef typename Type::value_type ScalarType;
		typedef Type RealDomainType;

		static type fromScalar(ScalarType scalar) { return type::from_value(scalar); }
		static ScalarType toScalar(type value) { return value.value(); }
	};

	/**
	 * Square root alias for primitive types.
	 */
	template <typename Type>
	static decltype(std::sqrt(Type())) sqrt(
		Type value,
		typename std::enable_if<
			std::is_floating_point<Type>::value or
			std::is_integral<Type>::value
		>::type* = 0
	) {
		return std::sqrt(value);
	}

	/**
	 * Square root alias for boost units.
	 */
	template <typename Type>
	static decltype(boost::units::sqrt(Type())) sqrt(
		Type value,
		typename std::enable_if<
			std::is_member_function_pointer<decltype(&Type::value)>::value
		>::type* = 0
	) {
		return boost::units::sqrt(value);
	}

	MemberType x;
	MemberType y;
};

template <typename MemberType>
FuVector<MemberType> operator+(MemberType value, FuVector<MemberType> vector) {
	return vector += value;
}

template <typename MemberType>
FuVector<MemberType> operator-(MemberType value, FuVector<MemberType> vector) {
	return vector -= value;
}

template <typename MemberType>
FuVector<MemberType> operator*(
	typename FuVector<MemberType>::ScalarType scalar,
	FuVector<MemberType> vector
) {
	return vector *= scalar;
}

/**
 * Regard the (2-dimensional) arguments as 3-dimensional vectors each with a
 * zeroed third component, calculate their cross product, and then return the
 * (only non-zero) third component of the result.
 *
 * This is used to calculate the sine between vectors.
 * It can also be used to determine torque in two dimensions.
 */
template <typename MemberType, typename OtherMemberType>
decltype(MemberType() * OtherMemberType()) pseudoCrossProduct(
	const FuVector<MemberType>& left,
	const FuVector<OtherMemberType>& right
) {
	return left.getX() * right.getY() - left.getY() * right.getX();
}

template <typename MemberType, typename OtherMemberType>
auto sineOfAngleBetween(
	const FuVector<MemberType>& left,
	const FuVector<OtherMemberType>& right
) -> decltype(pseudoCrossProduct(left, right) / (left.magnitude() * right.magnitude())) {
	auto operand1 = pseudoCrossProduct(left, right);
	auto operand2 = (left.magnitude() * right.magnitude());
	static_assert (
		std::is_floating_point<decltype(operand1 / operand2)>::value,
		"Must be a floating point division."
	);
	return operand1 / operand2;
}

template <typename MemberType, typename OtherMemberType>
auto cosineOfAngleBetween(
	const FuVector<MemberType>& left,
	const FuVector<OtherMemberType>& right
) -> decltype((left * right) / (left.magnitude() * right.magnitude())) {
	auto operand1 = left * right;
	auto operand2 = (left.magnitude() * right.magnitude());
	static_assert (
		std::is_floating_point<decltype(operand1 / operand2)>::value,
		"Must be a floating point division."
	);
	return operand1 / operand2;
}

template <typename MemberType, typename OtherMemberType>
Radian angleBetween(
	const FuVector<MemberType>& left,
	const FuVector<OtherMemberType>& right
) {
	return std::acos(cosineOfAngleBetween(left, right)) * radians;
}

#endif /* VECTOR_H_ */
