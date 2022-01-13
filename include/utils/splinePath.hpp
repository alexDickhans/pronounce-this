#pragma once

#include "path.hpp"
#include "pointUtil.hpp"
#include <vector>
#include <string>

namespace Pronounce {
	/**
	 * @brief Smooths a list of points using splines
	 * 
	 */
	class SplinePath {
	private:
		std::vector<Point> points;
	public:
		SplinePath();
		SplinePath(Path path);
		SplinePath(std::vector<Point> points);

		Point getPoint(double t, std::vector<Point> points) {
			if (t < 0 || t > 1) {
				throw "Invalid T value";
			}

			if (points.size() > 2) {
				std::vector<Point> newPoints;

				for (int i = 0; i < points.size() - 1; i++) {
					Point startPoint = points.at(i);
					Point endPoint = points.at(i + 1);
					
					newPoints.emplace_back(startPoint.lerp(endPoint, t));
				}

				return getPoint(t, newPoints);
			}

			Point startPoint = points.at(0);
			Point endPoint = points.at(1);

			return startPoint.lerp(endPoint, t);
		}

		Point getPoint(double t) {
			return getPoint(t, this->points);
		}


		Path getPath(double pointGranularity) {
			Path path;
			for (double i = 0; i <= 1; i += pointGranularity) {
				path.addPoint(this->getPoint(i));
			}
			return path;
		}

		void addPoint(Point point) {
			points.emplace_back(point);
		}

		void addPoint(double x, double y) {
			points.emplace_back(Point(x, y));
		}

		Point getPoint(int i) {
			return points.at(i);
		}

		std::string to_string() {
			std::string str = "";
			for (int i = 0; i < points.size(); i++) {
				str += points.at(i).to_string();
				if (i != points.size() - 1) {
					str += ", ";
				}
			}
			return str;
		}

		~SplinePath();
	};

	SplinePath::SplinePath(/* args */)
	{
	}

	SplinePath::SplinePath(Path path) {
		this->points = path.getPath();
	}

	SplinePath::SplinePath(std::vector<Point> points) {
		this->points = points;
	}

	SplinePath::~SplinePath()
	{
	}

} // namespace Pronounce
