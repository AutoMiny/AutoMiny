#include "boost/numeric/ublas/io.hpp"


#include "NewtonPolynomial.h"

NewtonPolynomial::NewtonPolynomial()
{
	dd = boost::numeric::ublas::matrix<double> (1, 1, 0);
	deg = -1;
}

NewtonPolynomial::NewtonPolynomial(double x, double y)
	: NewtonPolynomial()
{
	this->addData(x, y);
}

NewtonPolynomial::NewtonPolynomial(std::vector<FuPoint<double>> ps)
	: NewtonPolynomial()
{
	this->addData(ps);
}

NewtonPolynomial::~NewtonPolynomial()
{/* void */}

/**
 * adds a data-point to the polynomial and computes a proper coefficient.
 *
 * uses newtons divided differences to compute the coeeficients for a polynomial.
 *
 * the divided-differences are stored in a table to be more efficient.
 *
 * this breaks the table if to points with the same x value were given. FIXME
 *
 * @param x x-value of data-point
 * @param y y-value of data-point
 * @return pointer to this NewtonPolynomial
 * 
 * THE X and Y axes are swapped because we want to have the polynomial "verticaly"
 * only place where we need to swap it should be this place.
 */
NewtonPolynomial& NewtonPolynomial::addData(double y, double x)
{
	// assume that table for actual polynomial with given degree is complete

	// this is the nth point, which is the new degree of the polynomial
	int n = ++deg;

	// resize structures
	xs.resize(n+1);
	ys.resize(n+1);
	dd.resize(n+1, n+1);
	//TODO initialize table with zeros?

	// store given data
	xs[n] = x;
	ys[n] = y;

	// insert function output as diffTable[n][n]
	dd.insert_element(n, n, y);

	// compute new values for every diffTable[i][n] downwards
	double tmp;
	for (int i = n-1; i >= 0; --i)
	{

//		std::cout << "i: " << i << " n: " << n
//				<< " diffTable size: " << dd.size1() << " X " << dd.size2()
//				<< " xn: " << xs[n] << " xi: " << xs[i]
//				<< " dd(i+1,n): " << dd(i+1, n) << " dd(i,n-1): " << dd(i, n-1) << std::endl;

		tmp = (dd(i+1, n) - dd(i, n-1)) / (xs[n] - xs[i]);

		// coefficient is stored in diffTable(0, n)
		dd.insert_element(i, n, tmp);

//		std::cout << "divided-difference-table(" <<i <<")("<<n<<")" << dd(i, n) << " " << std::endl;
	}
//	std::cout << dd << std::endl;

	return *this;
}

/**
 * overloads addData(double x, double y)
 *
 * @param p the data-point to add
 * @return Pointer to this NewtonPolynomial
 */
NewtonPolynomial& NewtonPolynomial::addData(FuPoint<double> p)
{
	return this->addData(p.getX(), p.getY());
}

/**
 * overloads addData(double x, double y)
 *
 * @param p the data-point to add
 * @return Pointer to this NewtonPolynomial
 */
NewtonPolynomial& NewtonPolynomial::addDataXY(FuPoint<double> p)
{
	return this->addData(p.getY(), p.getX());
}

/**
 * this uses addData(double x, double y) to add the data points
 * of the given lists one after another
 *
 * @param ps data-points to add
 * @return Pointer to this polynomial
 */
NewtonPolynomial& NewtonPolynomial::addData(std::vector<FuPoint<double>> ps)
{
	for (FuPoint<double> p: ps)
	{
		this->addData(p.getX(), p.getY());
	}

	return *this;
}

/**
 * computes polynomial at position x
 *
 * this uses horners method for computation.
 * @param x input for polynomial
 * @return P(x)
 */
double NewtonPolynomial::at(double x) const
{
	int n = deg;

	// zero polynom is zero is zero is zero
	if (deg < 0)
//	if (!isInitialized())
		return 0.0;

	/*
	 * horner:
	 * P(x) = (..(c[n] * (x-x[n-1])+c[n-1])*(x-x[n-2])+..+ c[1])*(x-x[0])+c[0]
	 */
	double tmp = dd(0, n);	// c_n
	for (int i = 1; i <= n; ++i)
	{
		tmp *= (x - xs[n-i]);
		tmp += dd(0, n-i);
	}

//	std::cout << "used Coeffs: " << std::endl;
//	for (int i = 0; i <= n; ++i)
//	{
//		std::cout << dd(0, i) << ", ";
//	}
//	std::cout << std::endl;

	return tmp;
}

/**
 * the degree of the polynomial.
 * A degree of n means that there were n+1 datapoints added.
 * @return
 */
int NewtonPolynomial::getDegree() const
{
	return deg;
}

/**
 * when the degree is at least 0, the polynomial counts as initialized
 * @return
 */
bool NewtonPolynomial::isInitialized() const
{
	return deg >= 0;
}

/**
 * the calculated coefficients of the polynomial
 * @return
 */
std::vector<double> NewtonPolynomial::getCoefficients() const
{
    std::vector<double> coefficients;
    for (unsigned int i = 0; i < dd.size2(); i++) {
        coefficients.push_back(dd(0, i));
    }
    return coefficients;
}

/**
 * resets NewtonPolynomial to zero-polynomial
 */
NewtonPolynomial& NewtonPolynomial::clear()
{
	xs.clear();
	dd.clear();

	deg = -1;

	dd.resize(1, 1, false);
	dd.insert_element(0, 0, 0);

	return *this;
}
