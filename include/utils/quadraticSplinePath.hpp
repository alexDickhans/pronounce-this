#pragma once

#include "splinePoint.hpp"
#include "splinePath.hpp"
#include "path.hpp"
#include <vector>

namespace Pronounce
{
	class QuadraticSplinePath {
	private:
		std::vector<SplinePoint> points;
	public:
		QuadraticSplinePath();
		QuadraticSplinePath(std::vector<SplinePoint> points);

		Path getPath(double pointGranularity) {
			if (points.size() < 2) {
				throw "Not enough points";
			}

			Path path;
			for (int i = 0; i < points.size() - 1; i++) {
				printf("Index: %d\n", i);
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

				std::cout << "current path" << currentPath.to_string() << std::endl;
				
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

		~QuadraticSplinePath();
	};
	
	QuadraticSplinePath::QuadraticSplinePath(/* args */) {
	}

	QuadraticSplinePath::QuadraticSplinePath(std::vector<SplinePoint> points) {
		this->points = points;
	}
	
	QuadraticSplinePath::~QuadraticSplinePath() {
	}
	
} // namespace Pronounce
