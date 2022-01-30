
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
		bool continuePath = true;
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
				for (int i = 1; i < oldPath.size() - 1; i++) {
					Point oldPoint = oldPath.at(i);
					Point newPoint = path.at(i);

					// x
					double aux = path.at(i).getX();
					path.at(i).setX(path.at(i).getX()
						+ (weightData * (oldPoint.getX() - newPoint.getX()))
						+ (weightSmooth * (path.at(i - 1).getX() + path.at(i + 1).getX() - 2.0 * newPoint.getX()))
					);

					change += abs(aux - newPoint.getX());

					// y
					aux = path.at(i).getY();
					path.at(i).setY(path.at(i).getY()
						+ (weightData * (oldPoint.getY() - newPoint.getY()))
						+ (weightSmooth * (path.at(i - 1).getY() + path.at(i + 1).getY() - 2.0 * newPoint.getY()))
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

		double getTValue(Point currentPosition) {
			Point closestPoint;
			double closestDistance = INT32_MAX;
			double closestT = INT32_MAX;

			double totalT = 0;

			// Returns the largest item in list
			// If two items are the same distance apart, will return first one
			for (int i = 1; i < path.size(); i++) {
				Point lastPoint = path.at(i - 1);
				Point thisPoint = path.at(i);

				Vector thisMinusLast(&thisPoint, &lastPoint);
				Vector positionMinusLast(&currentPosition, &lastPoint);

				// https://diego.assencio.com/?index=ec3d5dfdfc0b6a0d147a656f0af332bd
				double t = positionMinusLast.dot(thisMinusLast) / thisMinusLast.dot(thisMinusLast);

				if (0 < t && t < 1) {
					lastPoint += Vector(&lastPoint, &thisPoint).scale(t).getCartesian();
				}
				else if (t > 1) {
					lastPoint = thisPoint;
				}
				else if (t < 0) {
					lastPoint = lastPoint;
				}

				double distance = lastPoint.distance(currentPosition);
				if (distance < closestDistance) {
					closestDistance = distance;
					closestPoint = lastPoint;
					closestT = totalT;
				}

				totalT += 1;
			}

			return closestT;
		}

		double distanceFromEnd(Point currentPosition) {
			double t = this->path.size() - 1 - this->getTValue(currentPosition);

			double total = 0;

			for (int i = path.size() - 1; i > 0 && t > 0; i ++) {
				Point startPoint = path.at(i);
				Point endPoint = path.at(i - 1);

				total += startPoint.distance(endPoint);

				if (t > 1) {
					t --;
				}
				else {
					t -= fmod(t, 1);
				}
			}

			return total;
		}

		double distanceFromStart(Point currentPosition) {
			double t = this->getTValue(currentPosition);

			double total = 0;

			for (int i = 1; i < path.size() - 1 && t > 0; i ++) {
				Point startPoint = path.at(i);
				Point endPoint = path.at(i - 1);

				total += startPoint.distance(endPoint);

				if (t > 1) {
					t --;
				}
				else {
					t -= fmod(t, 1);
				}
			}

			return total;
		}

		bool isContinuePath() {
			return continuePath;
		}

		void setContinuePath(bool continuePath) {
			this->continuePath = continuePath;
		}

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

		std::string to_string() {
			std::string str = "";
			for (Point point : path) {
				str += point.to_string() + "\n";
			}
			return str;
		}

		~Path();
	};
} // namespace Pronounce
