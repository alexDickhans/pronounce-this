//
// Created by alex on 8/30/23.
//

#include "polynomialExpression.hpp"

namespace Pronounce {
	PolynomialExpression::PolynomialExpression(const std::vector<double> &coefficients) {
		this->coefficients = coefficients;
	}

	PolynomialExpression::PolynomialExpression(const std::initializer_list<double> coefficients) {
		this->coefficients = coefficients;
	}

	PolynomialExpression PolynomialExpression::getDerivative() {
		if (coefficients.size() <= 1) {
			return PolynomialExpression({0.0});
		} else {
			std::vector<double> derivativeCoefficients;
			for (size_t i = 1; i < coefficients.size(); i++) {
				derivativeCoefficients.emplace_back(coefficients.at(i) * i);
			}
			return PolynomialExpression(derivativeCoefficients);
		}
	}

	double PolynomialExpression::evaluate(double x)  {
		double result = 0;

		for (int i = 0; i < coefficients.size(); i++) {
			result += pow(x, i) * coefficients.at(i);
		}

		return result;
	}

} // Pronounce