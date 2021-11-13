
#pragma once

#include <vector>
#include "pointUtil.hpp"
#include "vector.hpp"

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

		std::vector<Point> getPath() {
			return path;
		}

		void setPath(std::vector<Point> path) {
			this->path = path;
		}

		Point getPoint(size_t index) {
			return path.at(index);
		}

		void addPoint(Point point) {
			this->path.emplace_back(point);
		}

		void addPoint(double x, double y) {
			this->path.emplace_back(Point(x, y));
		}

		Point getLookAheadPoint(Point currentPosition, double lookaheadDistance);

		/**
		 * @brief Get the closest point in the path to the robot
		 *
		 * @param currentPosition
		 * @return Point The closest point
		 */
		Point getClosestPoint(Point currentPosition);

		~Path();
	};
} // namespace Pronounce
