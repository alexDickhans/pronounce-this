#pragma once

#include "point.hpp"
#include "vector.hpp"

// TODO: add docstrings
// TODO: add comments
// TODO: Change name

namespace Pronounce {
	class SplinePoint {
	private:
		Point point;
		Vector controlArm;
	public:
		SplinePoint();
		SplinePoint(Point point, Vector controlArm);
		SplinePoint(Point point, Point controlArm);

		Point getPoint() {
			return point;
		}

		void setPoint(const Point& newPoint) {
			this->point = newPoint;
		}

		Vector getControlArm() {
			return controlArm;
		}

		Point getControlArmPoint() {
			Point result = point;
			result.add(controlArm.getCartesian());
			return result;
		}

		Point getNegativeControlArm() {
			Point result = point;
			controlArm = Vector(controlArm);
			controlArm.rotate(M_PI);
			result.add(controlArm.getCartesian());
			return result;
		}

		void setControlArm(const Vector& newControlArm) {
			this->controlArm = newControlArm;
		}

		~SplinePoint();
	};
	
	SplinePoint::SplinePoint() {
		point = Point();
		controlArm = Vector();
	}

	SplinePoint::SplinePoint(Point point, Vector controlArm) {
		this->point = point;
		controlArm.rotate(M_PI_2);
		this->controlArm = controlArm;
	}
	
	SplinePoint::~SplinePoint()
	{
	}
} // namespace Pronounce
