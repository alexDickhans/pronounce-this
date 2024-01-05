#pragma once

#include "polynomialExpression.hpp"
#include "vector.hpp"
#include "linearInterpolator.hpp"

namespace PathPlanner {
	class BezierSegment {
	private:
		PolynomialExpression x, y, dx, dy, ddx, ddy;

		Point a;
		Point b;
		Point c;
		Point d;

		QLength length;
		LinearInterpolator distanceToT;

		bool reversed;

	public:
		BezierSegment(Point a, Point b, Point c, Point d, bool reversed = false, int granularity = 100) {
			this->a = a;
			this->b = b;
			this->c = c;
			this->d = d;

			this->reversed = reversed;

			x = PolynomialExpression({
				a.getX().getValue(),
				3.0*b.getX().getValue() - 3.0*a.getX().getValue(),
				3.0*c.getX().getValue() - 6.0*b.getX().getValue() + 3.0*a.getX().getValue(),
				d.getX().getValue() - 3.0*c.getX().getValue() + 3.0*b.getX().getValue() - a.getX().getValue()}
				);
			y = PolynomialExpression({
				 a.getY().getValue(),
				 3.0*b.getY().getValue() - 3.0*a.getY().getValue(),
				 3.0*c.getY().getValue() - 6.0*b.getY().getValue() + 3.0*a.getY().getValue(),
				 d.getY().getValue() - 3.0*c.getY().getValue() + 3.0*b.getY().getValue() - a.getY().getValue()}
				 );

			dx = x.getDerivative();
			dy = y.getDerivative();

			ddx = dx.getDerivative();
			ddy = dy.getDerivative();

			length = 0.0;
			distanceToT.add(0, 0);

			for (int t = 0; t < granularity; t ++) {
				length += sqrt(pow(dx.evaluate((double)t/(double) granularity), 2) + pow(dy.evaluate((double)t/(double) granularity), 2));
				length += sqrt(pow(dx.evaluate((double)(t+1)/(double) granularity), 2) + pow(dy.evaluate((double)(t+1)/(double) granularity), 2));
				distanceToT.add(0.5 * (1.0/(double) granularity) * length.getValue(), (double)(t+1)/(double) granularity);
			}

			length = 0.5 * (1.0/(double) granularity) * length;
			std::cout << "CALC-len " << length.Convert(inch) << std::endl;
		}

		QLength getDistance() {
			return length.getValue() * (reversed ? -1.0 : 1.0);
		}

		double getTByLength(QLength distance) {
			return distanceToT.get(distance.getValue());
		}

		QCurvature getCurvature(double t) {
			return -(dx.evaluate(t)*ddy.evaluate(t) - ddx.evaluate(t)*dy.evaluate(t))/pow(Vector(Point(dx.evaluate(t), dy.evaluate(t))).getMagnitude().getValue(), 3);
		}

		Angle getAngle(double t) {
			return -atan2(dy.evaluate(t), dx.evaluate(t)) * radian + 90_deg;
		}

		Point evaluate(double t) {
			return {x.evaluate(t), y.evaluate(t)};
		}

		double getSpeedMultiplier(double t, QLength trackWidth) {
			QCurvature maxCurvature = this->getCurvature(t);

			if (maxCurvature.getValue() == 0.0)
				return 1.0;

			return 1.0/(1.0 + abs(maxCurvature.getValue() * 0.5) * trackWidth.getValue());
		}

		Point getA() {
			return a;
		}

		Point getB() {
			return b;
		}

		Point getC() {
			return c;
		}

		Point getD() {
			return d;
		}

		bool getReversed() const {
			return reversed;
		}
	};
}
