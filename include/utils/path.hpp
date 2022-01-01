
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

		void fill(double spacing) {
			std::vector<Point> oldPath = path;
			path.clear();

			for (int i = 0; i < oldPath.size() - 1; i++) {
				// Fill every point in the path with the spacing
				Point point = oldPath.at(i);
				Vector vector = Vector(&point, &oldPath.at(i + 1));
				double pointCount = ceil(vector.getMagnitude() / spacing);
				vector.normalize();
				vector = vector.scale(spacing);

				for (int i = 0; i < pointCount - 1; i++) {
					point.add(vector.getCartesian());
					path.emplace_back(point);
				}

				path.emplace_back(oldPath.at(i + 1));
			}
		}
		
		/**
		 * @brief Smooth the path
		 * 
		 * @deprecated Not working yet
		 * 
		 * @param weightSmooth 
		 * @param tolerance 
		 */
		void smooth(double weightSmooth, double tolerance) {
			double weightData = 1 - weightSmooth;

			std::vector<Point> oldPath = path;

			double change = tolerance;

			while (change >= tolerance) {
				change = 0.0;
				for (int i = 1; i < oldPath.size()-1; i++) {
					Point oldPoint = oldPath.at(i);
					Point newPoint = path.at(i);

					// x
					double aux = path.at(i).getX();
					path.at(i).setX(path.at(i).getX()
									 + (weightData * (oldPoint.getX() - newPoint.getX()))
									 + (weightSmooth * (path.at(i-1).getX() + path.at(i+1).getX() - 2.0 * newPoint.getX()))
									 );

					change += abs(aux - newPoint.getX());

					// y
					aux = path.at(i).getY();
					path.at(i).setY(path.at(i).getY()
									 + (weightData * (oldPoint.getY() - newPoint.getY()))
									 + (weightSmooth * (path.at(i-1).getY() + path.at(i+1).getY() - 2.0 * newPoint.getY()))
									 );
					
					change += abs(aux - newPoint.getY());
				}
			}
		}

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

		void operator+= (Point point) {
			this->path.emplace_back(point);
		}

		void operator+= (std::vector<Point> points) {
			for (Point point : points) {
				this->path.emplace_back(point);
			}
		}

		void operator+= (Path path) {
			for (Point point : path.getPath()) {
				this->path.emplace_back(point);
			}
		}

		~Path();
	};
} // namespace Pronounce
