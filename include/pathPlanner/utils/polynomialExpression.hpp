//
// Created by alex on 8/30/23.
//
#pragma once

#include <vector>

namespace PathPlanner {

	class PolynomialExpression {
	private:
		std::vector<double> coefficients;
	public:
		PolynomialExpression() = default;
		explicit PolynomialExpression(const std::vector<double>& coefficients) {
			this->coefficients = coefficients;
		}
		PolynomialExpression(const std::initializer_list<double> coefficients) {
			this->coefficients = coefficients;
		}

		PolynomialExpression getDerivative() {
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

		double evaluate(double x) const {
			double result = 0.0;

			for (int i = 0; i < coefficients.size(); i++) {
				result += pow(x, i) * coefficients.at(i);
			}

			return result;
		}

		~PolynomialExpression() = default;
	};

} // Pronounce
