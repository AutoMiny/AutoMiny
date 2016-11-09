#ifndef FuPoint_H_
#define FuPoint_H_

#include <string>
#include <ostream>

#include "FuVector.h"

/**
 * A (2-dimensional) FuPoint template, represented by its origin vector.
 *
 * Arithmetic operators are also defined to relate these FuPoints with vectors:
 * - subtracting a FuPoint from another FuPoint yields a FuVector
 * - adding a FuPoint and a FuVector yields a FuPoint
 * - adding a FuPoint to another FuPoint is not allowed
 * (like in school math)
 */
template <typename MemberType>
class FuPoint {
public:
	FuPoint() : FuPoint(FuVector<MemberType>()) { /* void */ }
	FuPoint(FuVector<MemberType> originFuVector) : originFuVector(originFuVector) { /* void */ }
	FuPoint(MemberType x, MemberType y) : originFuVector(x, y) { /* void */ }

	virtual ~FuPoint() { /* void */ }

	const FuVector<MemberType>& getOriginFuVector() const { return originFuVector; }
	MemberType getX() const { return originFuVector.getX(); }
	MemberType getY() const { return originFuVector.getY(); }

	std::string toString() const {
		std::stringstream result;
		result << "FuPoint {" << getX().value() << ", " << getY().value() << "}";
		return result.str();
	}

	FuPoint<MemberType> operator+(const FuVector<MemberType>& vector) const {
		return FuPoint<MemberType>(*this) += vector;
	}

	FuPoint<MemberType> operator-(const FuVector<MemberType>& vector) const {
		return FuPoint<MemberType>(*this) -= vector;
	}

	FuVector<MemberType> operator-(const FuPoint<MemberType>& other) const {
		return this->originFuVector - other.originFuVector;
	}

	bool operator==(const FuPoint& other) const {
		return this->originFuVector == other.originFuVector;
	}

	bool operator!=(const FuPoint& other) const {
		return !(*this == other);
	}

	FuPoint<MemberType>& operator+=(const FuVector<MemberType>& vector) {
		this->originFuVector += vector;
		return *this;
	}

	FuPoint<MemberType>& operator-=(const FuVector<MemberType>& vector) {
		this->originFuVector -= vector;
		return *this;
	}

	friend std::ostream& operator<<(std::ostream& stream, const FuPoint<MemberType>& FuPoint) {
		stream << FuPoint.getOriginFuVector();
		return stream;
	}

	friend std::istream& operator>>(std::istream& stream, FuPoint<MemberType>& FuPoint) {
		stream >> FuPoint.originFuVector;
		return stream;
	}

private:
	FuVector<MemberType> originFuVector;
};

template <typename MemberType>
FuPoint<MemberType> operator+(const FuVector<MemberType>& left, const FuPoint<MemberType>& right) {
	return right + left;
}

template <typename MemberType>
MemberType distance(const FuPoint<MemberType>& FuPoint1, const FuPoint<MemberType>& FuPoint2) {
	return (FuPoint2 - FuPoint1).magnitude();
}

template <typename MemberType>
FuPoint<MemberType> center(const FuPoint<MemberType>& FuPoint1, const FuPoint<MemberType>& FuPoint2) {
	return FuPoint1 + (FuPoint2 - FuPoint1) / 2;
}

#endif /* FuPoint_H_ */
