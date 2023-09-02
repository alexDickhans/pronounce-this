#pragma once

#include "path.hpp"
#include "pointUtil.hpp"
#include "polynomialExpression.hpp"
#include <utility>
#include <vector>
#include <string>
#include <cmath>

// TODO: add docstrings
// TODO: add comments
// TODO: Change name

namespace Pronounce {

	class Spline {
	private:
		PolynomialExpression polynomial;
	public:
		explicit Spline(const PolynomialExpression& polynomialExpression) {
			polynomial = polynomialExpression;
		}

		Spline(const std::initializer_list<double> points) {
			this->calculatePolynomial(points);
		}

		explicit Spline(const std::vector<double>& points) {
			this->calculatePolynomial(points);
		}

		void calculatePolynomial(const std::vector<double>& points) {
			std::vector<double> coefficients;

			for (int i = 0; i < points.size()-1; i++) {
				unsigned int scalar = factorial(i);

				coefficients.emplace_back(0);
			}

			polynomial = PolynomialExpression(coefficients);
		}

		double calculateValue(double t) {
			return polynomial.evaluate(t);
		}

		Spline getDerivative() {
			return Spline(polynomial.getDerivative());
		}
	};

	/**
	 * @brief Smooths a list of points using splines
	 * 
	 */
	class SplinePath {
	private:
		std::vector<Point> points;
	public:
		SplinePath();
		explicit SplinePath(Path path);
		explicit SplinePath(std::vector<Point> points);

		static Point getPoint(double t, std::vector<Point> points) {
			if (t < 0 || t > 1) {
				throw "Invalid T value";
			}

			if (points.size() > 2) {
				std::vector<Point> newPoints;

				for (int i = 0; i < points.size() - 1; i++) {
					Point startPoint = points.at(i);
					Point endPoint = points.at(i + 1);
					
					newPoints.emplace_back(startPoint.lerpPoint(endPoint, t));
				}

				return getPoint(t, newPoints);
			}

			Point startPoint = points.at(0);
			Point endPoint = points.at(1);

			return startPoint.lerpPoint(endPoint, t);
		}

		Point getPoint(double t) {
			return getPoint(t, this->points);
		}

		Path getPath(double pointGranularity) {
			Path path;
			for (double i = 0; i <= 1; i += pointGranularity) {
				path.addPoint(this->getPoint(i));
			}
			return path;
		}

		void addPoint(const Point& point) {
			points.emplace_back(point);
		}

		void addPoint(double x, double y) {
			points.emplace_back(x, y);
		}

		Point getPoint(int i) {
			return points.at(i);
		}

		std::string to_string() {
			std::string str;
			for (int i = 0; i < points.size(); i++) {
				str += points.at(i).to_string();
				if (i != points.size() - 1) {
					str += ", ";
				}
			}
			return str;
		}

		Angle getOrientation(double t) {
			return 0.0;
		}

		QCurvature getCurvatureAtT(double t) {

		}

		~SplinePath();
	};

	SplinePath::SplinePath(/* args */) = default;

	SplinePath::SplinePath(Path path) {
		this->points = path.getPath();
	}

	SplinePath::SplinePath(std::vector<Point> points) {
		this->points = std::move(points);
	}

	SplinePath::~SplinePath()
	{
	}

} // namespace Pronounce
