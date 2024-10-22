#pragma once

#include "polynomialExpression.hpp"
#include "linearInterpolator.hpp"
#include "velocityProfile/velocityProfile.hpp"

namespace PathPlanner {
	class BezierSegment {
	protected:
		PolynomialExpression x, y, dx, dy, ddx, ddy;

		Point a;
		Point b;
		Point c;
		Point d;

		QLength length;
		LinearInterpolator distanceToT;
		Pronounce::ProfileConstraints profileConstraints;

		bool reversed;
		bool stopEnd;

	public:
		BezierSegment(Point a, Point b, Point c, Point d, bool reversed, bool stopEnd, const Pronounce::ProfileConstraints &profileConstraints, int granularity = 100) : profileConstraints(profileConstraints) {
			this->a = a;
			this->b = b;
			this->c = c;
			this->d = d;

			this->reversed = reversed;
			this->stopEnd = stopEnd;

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

			for (int t = 0.0; t <= granularity; t += 1) {
				length += sqrt(pow(dx.evaluate((double)t/(double) granularity), 2) + pow(dy.evaluate((double)t/(double) granularity), 2)) / (double) granularity;
				distanceToT.add(length.getValue(), (double)t/(double) granularity);
			}
		}


		[[nodiscard]] virtual QLength getDistance() const {
			return length.getValue();
		}

		[[nodiscard]] double getTByLength(QLength distance) const {
			return distanceToT.get(distance.getValue());
		}

		[[nodiscard]] QCurvature getCurvature(double t) const {
			return (reversed ? 1.0 : -1.0) * (dx.evaluate(t)*ddy.evaluate(t) - ddx.evaluate(t)*dy.evaluate(t))/pow(sqrt(pow(dx.evaluate(t), 2) + pow(dy.evaluate(t), 2)), 3);
		}

		[[nodiscard]] QCurvature getMaxCurvature(int granularity = 20) const {
			QCurvature maxCurvature = 0.0;

			for (size_t i = 0; i <= granularity; i++) {
				QCurvature curvature = this->getCurvature(static_cast<double>(i)/granularity).getValue();
				maxCurvature = std::max(abs(curvature.getValue()), maxCurvature.getValue());
			}

			return maxCurvature;
		}

		[[nodiscard]] Angle getAngle(double t) const {
			return -atan2(dy.evaluate(t), dx.evaluate(t)) * radian + 90_deg;
		}

		[[nodiscard]] Point evaluate(double t) const {
			return {x.evaluate(t), y.evaluate(t)};
		}

		[[nodiscard]] double getMaxSpeedMultiplier(QLength trackWidth, int granularity = 100) const {
			QCurvature maxCurvature = this->getMaxCurvature(granularity);

			if (maxCurvature.getValue() == 0.0)
				return 1.0;

			return 1.0/(1.0 + abs(maxCurvature.getValue() * 0.5) * trackWidth.getValue());
		}

		[[nodiscard]] double getMaxSpeedMultiplier(QLength trackWidth, double t) const {
			QCurvature curvature = this->getCurvature(t);

			if (curvature.getValue() == 0.0)
				return 1.0;

			return 1.0/(1.0 + abs(curvature.getValue() * 0.5) * trackWidth.getValue());
		}

		[[nodiscard]] const Point &getA() const {
			return a;
		}

		[[nodiscard]] const Point &getB() const {
			return b;
		}

		[[nodiscard]] const Point &getC() const {
			return c;
		}

		[[nodiscard]] const Point &getD() const {
			return d;
		}

		[[nodiscard]] bool isReversed() const {
			return reversed;
		}

		[[nodiscard]] const Pronounce::ProfileConstraints &getProfileConstraints() const {
			return profileConstraints;
		}

		void setProfileConstraints(const Pronounce::ProfileConstraints &profileConstraints) {
			this->profileConstraints = profileConstraints;
		}

		[[nodiscard]] bool isStopEnd() const {
			return stopEnd;
		}

		~BezierSegment() = default;
	};
}
