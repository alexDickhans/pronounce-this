
#pragma once

#include <vector>
#include "pointUtil.hpp"
#include "vector.hpp"
#include "utils.hpp"
#include <string>

namespace Pronounce {

	/**
	 * @brief List of points to use in pure pursuit tracking algorithm
	 *	
	 * @authors Alex Dickhans (alexDickhans)
	 */
	class Path {
	private:
		/**
		 * @brief List of points in the path
		 * 
		 */
		std::vector<Point> path;

		/**
		 * @brief To continue the line past the end of the path, useful for odom
		 * 
		 */
		bool continuePath = true;

		/**
		 * @brief Name of the path, used for debugging
		 * 
		 */
		std::string name = "";
	public:
		/**
		 * @brief Construct a new Path object with blank path and name
		 * 
		 */
		Path();

		/**
		 * @brief Construct a new Path object with blank path
		 * 
		 * @param name Name of the path to be used for debugging
		 */
		Path(std::string name);

		/**
		 * @brief Fill in the path with points
		 * 
		 * @param spacing The points to inject per line
		 */
		void fill(QLength spacing) {
			std::vector<Point> oldPath = path;
			path.clear();

			for (int i = 0; i < oldPath.size() - 1; i++) {
				// Fill every point in the path with the spacing
				Point point = oldPath.at(i);
				Vector vector = Vector(&point, &oldPath.at(i + 1));
				double pointCount = ceil((vector.getMagnitude() / spacing).getValue());
				vector.normalize();
				vector = vector.scale(spacing.getValue());

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
					QLength aux = path.at(i).getX();
					path.at(i).setX(path.at(i).getX()
						+ (weightData * (oldPoint.getX() - newPoint.getX()))
						+ (weightSmooth * (path.at(i - 1).getX() + path.at(i + 1).getX() - 2.0 * newPoint.getX()))
					);

					change += abs((aux - newPoint.getX()).getValue());

					// y
					aux = path.at(i).getY();
					path.at(i).setY(path.at(i).getY()
						+ (weightData * (oldPoint.getY() - newPoint.getY()))
						+ (weightSmooth * (path.at(i - 1).getY() + path.at(i + 1).getY() - 2.0 * newPoint.getY()))
					);

					change += abs((aux - newPoint.getX()).getValue());
				}
			}
		}

		/**
		 * @brief Get the Path object
		 * 
		 * @return std::vector<Point> List of points in the path
		 */
		std::vector<Point> getPath() {
			return path;
		}

		/**
		 * @brief Set the Path object
		 * 
		 * @param path List of points in the new path
		 */
		void setPath(std::vector<Point> path) {
			this->path = path;
		}

		/**
		 * @brief Get a point in the path
		 * 
		 * @param index Index of the point
		 * @return Point Point at that index
		 */
		Point getPoint(size_t index) {
			return path.at(index);
		}

		/**
		 * @brief Add a point to the path
		 * 
		 * @param point Point to add
		 */
		void addPoint(Point point) {
			this->path.emplace_back(point);
		}

		/**
		 * @brief Add a point to the path with x and y
		 * 
		 * @param x X distance
		 * @param y Y distance
		 */
		void addPoint(QLength x, QLength y) {
			this->path.emplace_back(Point(x, y));
		}

		/**
		 * @brief Get the current look ahead point
		 * 
		 * @param currentPosition The current position of the robot
		 * @param lookaheadDistance The lookahead distance
		 * @return Point The point at that lookahead distance
		 */
		Point getLookAheadPoint(Point currentPosition, QLength lookaheadDistance);

		/**
		 * @brief Get the closest point in the path to the robot
		 *
		 * @param currentPosition
		 * @return Point The closest point
		 */
		Point getClosestPoint(Point currentPosition);

		/**
		 * @brief Get the t value of the closest point
		 * 
		 * @param currentPosition The current position of the robot
		 * @return double The t value of the closest position
		 */
		double getTValue(Point currentPosition) {
			Point closestPoint;
			QLength closestDistance = (double) INT32_MAX;
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

				QLength distance = lastPoint.distance(currentPosition);
				if (distance < closestDistance) {
					closestDistance = distance;
					closestPoint = lastPoint;
					closestT = totalT + t;
				}

				totalT += 1;
			}

			return closestT;
		}

		/**
		 * @brief Get the total distance of the path
		 * 
		 * @return QLength Total distance of the path
		 */
		QLength distance() {
			QLength total = 0.0;

			for (int i = 1; i < path.size(); i++) {
				Point startPoint = path.at(i-1);
				Point endPoint = path.at(i);

				total += startPoint.distance(endPoint);
			}

			return total;
		}

		/**
		 * @brief Get the distance of the robot from the start of the path to the t value
		 * 
		 * @param t The t value that you want to get the distance at 
		 * @return QLength The current length of the path
		 */
		QLength distanceFromStart(double t) {
			double t2 = t;

			QLength total = 0.0;

			for (int i = 1; i <= std::min(ceil(t), path.size()-1.0); i ++) {
				Point startPoint = path.at(i-1);
				Point endPoint = path.at(i);

				total += startPoint.distance(endPoint).getValue() * clamp(t2, 0.0, 1.0);
				t2--;
			}

			return total;
		}

		/**
		 * @brief Get the distance from the start of the path
		 * 
		 * @param currentPosition The current position of the robot
		 * @return QLength The distance from the start of the path
		 */
		QLength distanceFromStart(Point currentPosition) {
			return this->distanceFromStart(this->getTValue(currentPosition));
		}

		/**
		 * @brief The distance from the end of the path
		 * 
		 * @param currentPosition The current position of the robot
		 * @return QLength The distance from the end of the path
		 */
		QLength distanceFromEnd(Point currentPosition) {
			return this->distance() - this->distanceFromStart(currentPosition);
		}

		/**
		 * @brief Get if the continue path variable
		 * 
		 * @return true If the continue path is true
		 * @return false If the continue path is false
		 */
		bool isContinuePath() {
			return continuePath;
		}

		/**
		 * @brief Set the Continue Path object
		 * 
		 * @param continuePath The new continue path object
		 */
		void setContinuePath(bool continuePath) {
			this->continuePath = continuePath;
		}

		/**
		 * @brief Get the name of the object
		 * 
		 * @return std::string predefined name of the object
		 */
		std::string getName() {
			return name;
		}

		/**
		 * @brief Set the Name of the object
		 * 
		 * @param name New name of the object, used for debugging
		 */
		void setName(std::string name) {
			this->name = name;
		}

		/**
		 * @brief Add a point to the path
		 * 
		 * @param point Point to add to the path
		 */
		void operator+= (Point point) {
			this->path.emplace_back(point);
		}

		/**
		 * @brief Add a vector of points to the path
		 * 
		 * @param points List of points to add to the path
		 */
		void operator+= (std::vector<Point> points) {
			for (Point point : points) {
				this->path.emplace_back(point);
			}
		}

		/**
		 * @brief Add 2 paths together
		 * 
		 * @param path Path to add
		 */
		void operator+= (Path path) {
			for (Point point : path.getPath()) {
				this->path.emplace_back(point);
			}
		}

		/**
		 * @brief Formatted string of the values in the path
		 * 
		 * @return std::string Formatted strings
		 */
		std::string to_string() {
			std::string str = "Name: " + name + ",\n Path: \n";
			for (Point point : path) {
				str += " " + point.to_string() + "\n";
			}
			return str;
		}

		~Path();
	};
} // namespace Pronounce
