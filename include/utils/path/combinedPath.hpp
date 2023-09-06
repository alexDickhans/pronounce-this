#pragma once
//
// Created by alex on 9/5/23.
//


namespace Pronounce {
	typedef struct PathSegment_ {
		QLength distance;
		QCurvature curvature;
	} PathSegment;

	class CombinedPath {
	private:
		std::vector<PathSegment> path;

		size_t getIndexAtT(double t) {
			return std::floor(t * (double) path.size());
		}

		double getRemainderAtT(double t) {
			return t-getIndexAtT(t) * path.size();
		}
	public:
		CombinedPath() {
			path = std::vector<PathSegment>();
		}

		CombinedPath(std::initializer_list<PathSegment> path) {
			this->path = path;
		}

		explicit CombinedPath(const std::vector<PathSegment>& path) {
			this->path = path;
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

			return 0.0;
		}

		QLength getDistanceAtT(double t) {
			if (t > 1 || t < 0) {
				return 0.0;
			}

			QLength distance = 0.0;

			size_t index = this->getIndexAtT(t);
			double remainder = this->getRemainderAtT(t);

			for (int i = 0; i < index; i++) {
				distance += path[i].distance;
			}

			distance += remainder * path[index].distance;

			return distance;
		}

		QLength getDistance() {
			QLength distance;

			for (auto segment : path) {
				distance += segment.distance;
			}

			return distance;
		}

		QCurvature getCurvatureAtDistance(QLength distance) {
			return getCurvatureAtT(getTAtDistance(distance));
		}

		QCurvature getCurvatureAtT(double t) {
			if (t > 1 || t < 0) {
				return 0.0;
			}
			return this->getSegmentAtT(t).curvature;
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


		}

		void addPathSegment(PathSegment pathSegment) {
			path.emplace_back(pathSegment);
		}
	};

	class PathBuilder {
	private:
		CombinedPath path;
	public:
		PathBuilder() {
			path = CombinedPath();
		};

		PathBuilder addSegment(PathSegment pathSegment) {
			path.addPathSegment(pathSegment);
		}

		PathBuilder addSegment(QLength distance, QCurvature curvature) {
			path.addPathSegment({distance, curvature});
		}

		CombinedPath build() {
			return path;
		}
	};
} // Pronounce
