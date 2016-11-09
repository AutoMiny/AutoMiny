#ifndef NEWTONPOLYNOMIAL_H_
#define NEWTONPOLYNOMIAL_H_

#include <boost/numeric/ublas/matrix.hpp>

#include "FuPoint.h"

class NewtonPolynomial {
public:

	NewtonPolynomial();
	NewtonPolynomial(double x, double y);
	NewtonPolynomial(FuPoint<double> p);
	NewtonPolynomial(std::vector<FuPoint<double>> ps);
	virtual ~NewtonPolynomial();

	NewtonPolynomial& addData(double x, double y);
	NewtonPolynomial& addData(FuPoint<double> p);
	NewtonPolynomial& addDataXY(FuPoint<double> p);
	NewtonPolynomial& addData(std::vector<FuPoint<double>> ps);

	double at(double x) const;

	int getDegree() const;
	bool isInitialized() const;
	std::vector<double> getCoefficients() const;

	NewtonPolynomial& clear();

private:
	/**
	 * this stores the given data
	 */
	std::vector<double> xs, ys;

	/**
	 * this is a divided-difference table
	 * it is undefined for i>j: diffTable[i][j]
	 * for i == j: dataPoints[i].y = diffTable[i][i]
	 * diffTable[0] gives the computed coefficients
	 */
	boost::numeric::ublas::matrix<double> dd;

	/**
	 * degree of the polynomial
	 * zero polynomial has a degree of -1
	 */
	int deg;

};

#endif /* NEWTONPOLYNOMIAL_H_ */
