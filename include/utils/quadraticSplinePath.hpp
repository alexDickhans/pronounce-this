#pragma once

#include "splinePoint.hpp"
#include "splinePath.hpp"
#include "path.hpp"
#include <vector>

// TODO: add docstrings
// TODO: add comments

namespace Pronounce {
	class BezierPath : public Path {
	private:
		std::vector<SplinePoint> points;
		std::string name = "";
	public:
		BezierPath();
		BezierPath(std::vector<SplinePoint> points);
		BezierPath(std::string name);

		Path getPath(double pointGranularity) {
			if (points.size() < 2) {
				throw "Not enough points";
			}

			Path path(name);
			
			for (int i = 0; i < points.size() - 1; i++) {
				SplinePoint startingSplinePoint = points.at(i);
				SplinePoint endingSplinePoint = points.at(i + 1);

				Point startingPoint = startingSplinePoint.getPoint();
				Point startingControlPoint = startingSplinePoint.getControlArmPoint();
				Point endingPoint = endingSplinePoint.getPoint();
				Point endingControlPoint = endingSplinePoint.getNegativeControlArm();

				SplinePath currentPath;
				currentPath.addPoint(startingPoint);
				currentPath.addPoint(startingControlPoint);
				currentPath.addPoint(endingControlPoint);
				currentPath.addPoint(endingPoint);
				
				path += currentPath.getPath(pointGranularity);
			}

			return path;
		}
		
		std::vector<SplinePoint> getPoints() {
			return points;
		}

		void addPoint(SplinePoint point) {
			points.emplace_back(point);
		}

		~BezierPath();
	};
	
	BezierPath::BezierPath(/* args */) {
	}

	BezierPath::BezierPath(std::string name) {
		this->name = name;
	}

	BezierPath::BezierPath(std::vector<SplinePoint> points) {
		this->points = points;
	}
	
	BezierPath::~BezierPath() {
	}
	
} // namespace Pronounce
