#pragma once

#include <vector>
#include "point.hpp"

namespace Pronounce {

	/**
	 * @brief List of points to use in pure pursuit tracking algorithm
	 *
	 */
	class Path {
	private:
		std::vector<Point> path;
	public:
		Path();

		std::vector<Point> setPath() {
			
		}

		Point getPoint(size_t index) {
			return path.at(index);
		}

		Point getLookAheadPoint(Point currentPosition, double lookaheadDistance) {

		}

		/**
		 * @brief Get the closest point in the path to the robot
		 *
		 * @param currentPosition
		 * @return Point The closest point
		 */
		Point getClosestPoint(Point currentPosition);

		~Path();
	};

	Path::Path(/* args */) {
	}

	Point Path::getClosestPoint(Point currentPosition) {
		Point closestPoint;
		double closestDistance = 0;

		// Returns the largest item in list
		// If two items are the same distance apart, will return first one
		for (int i = 0; i < path.size(); i++) {
			double distance = path.at(i).distance(currentPosition);
			if (distance < closestDistance) {
				closestDistance = distance;
				closestPoint = path.at(i);
			}
		}

		return closestPoint;
	}

	Path::~Path() {
	}

} // namespace Pronounce
