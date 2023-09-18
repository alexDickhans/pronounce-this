#pragma once

#include <utility>
#include <vector>
#include <algorithm>
#include "units/units.hpp"

namespace Pronounce {
	typedef struct PathSegment_ {
		QLength distance;
		QCurvature curvature;
	} PathSegment;

	class CombinedPath {
	private:
		std::vector<PathSegment> path;
		bool inverted{false};

		size_t getIndexAtT(double t) {
			if (t < 0 || t > 1) {
				throw std::exception();
			}
			std::cout << "HII " << t;
			size_t result = fmin((size_t) (t * (double) path.size()), path.size()-1);
			return result;
		}

		double getRemainderAtT(double t) {
			if (t < 0 || t > 1) {
				throw "get Bad t value";
			}
			return fmod(t, 1.0/(double) path.size()) * (double) path.size();
		}
	public:
		CombinedPath() {
			path = std::vector<PathSegment>();
		}

		CombinedPath(std::initializer_list<PathSegment> path) {
			this->path = path;
			if (this->path[0].distance < 0.0_in) {
				inverted = true;
			}

			for (auto &item: this->path) {
				item = {fabs(item.distance.getValue()), item.curvature};
			}
		}

		explicit CombinedPath(const std::vector<PathSegment>& path) {
			this->path = path;
			if (this->path[0].distance < 0.0_in) {
				inverted = true;
			}

			for (auto &item: this->path) {
				item = {fabs(item.distance.getValue()), item.curvature};
			}
		}

		PathSegment getSegmentAtT(double t) {
			return path.at(this->getIndexAtT(t));
		}

		PathSegment getSegmentAtDistance(QLength distance) {
			return getSegmentAtT(getTAtDistance(distance));
		}

		double getTAtDistance(QLength distance) {
			if (distance > this->getDistance() || distance < 0_m) {
				return 0.0;
			}

			size_t i = 0;

			while (distance > path.at(i).distance) {
				distance -= path.at(i).distance;
				i++;
			}

//			std::cout << " result: " << (double) i/(double) path.size() + (distance/path.at(i).distance).getValue()/(double) path.size() << std::endl;

			return (double) i/(double) path.size() + (distance/path.at(i).distance).getValue()/(double) path.size();
		}

		Angle getAngleAtDistance(QLength distance) {
			return getAngleAtT(getTAtDistance(distance));
		}

		Angle getAngleAtT(double t) {
			size_t index = this->getIndexAtT(t);
			double remainder = this->getRemainderAtT(t);

			if (t == 1.0)
				index = path.size();

			Angle totalAngle = 0.0;

			for (int i = 0; i < index; i++) {
				totalAngle += path.at(i).curvature * path.at(i).distance;
			}

			return totalAngle.getValue()  +
				(index != path.size() ?
					(path.at(index).distance.getValue() * path.at(index).curvature.getValue() * remainder)
					: 0.0);
		}

		QLength getDistance() {
			QLength distance;

			for (auto segment : path) {
				distance += segment.distance;
			}

			return distance;
		}

		QCurvature getMaxCurvature() {
			QCurvature maxCurvature = 0.0;

			std::for_each(path.begin(), path.end(), [&](const auto &item) {
				maxCurvature = std::max(maxCurvature.getValue(), item.curvature.getValue());
			});

			return maxCurvature;
		}

		double getSpeedMultiplier(QLength trackWidth) {
			QCurvature maxCurvature = this->getMaxCurvature();

			if (maxCurvature.getValue() == 0.0)
				return 1.0;

			return 1.0/(1.0 + abs(maxCurvature.getValue() * 0.5) * trackWidth.getValue());
		}

		void addPathSegment(PathSegment pathSegment) {
			path.emplace_back(pathSegment);
		}

		bool getInverted() {
			return this->inverted;
		}
	};
} // Pronounce
