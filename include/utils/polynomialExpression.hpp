#pragma once

#include <vector>
#include <cmath>

namespace Pronounce {

	class PolynomialExpression {
	private:
		std::vector<double> coefficients;
	public:
		PolynomialExpression() = default;
		explicit PolynomialExpression(const std::vector<double>& coefficients);
		PolynomialExpression(const std::initializer_list<double> coefficients);

		PolynomialExpression getDerivative();

		double evaluate(double x);

		PolynomialExpression& operator=(const PolynomialExpression& polynomialExpression) = default;
	};

} // Pronounce
