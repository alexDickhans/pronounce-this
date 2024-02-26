#pragma once

#include "locolib/localization/particleFilter/particle.hpp"
#include <limits>
#include "pros/gps.hpp"

namespace Loco {
	class FieldModel {
		std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> walls;
		std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> lines;
		QLength lineThickness;
	public:
		FieldModel(std::initializer_list<std::pair<Eigen::Vector2d, Eigen::Vector2d>> walls, std::initializer_list<std::pair<Eigen::Vector2d, Eigen::Vector2d>> lines, QLength lineThickness) {
			this->walls = walls;
			this->lines = lines;
			this->lineThickness = lineThickness;
		}

		// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
		QLength getDistanceToObstacle(const Particle& particle) {
			Eigen::Vector2d point1 = particle.getState().block<2, 1>(0, 0);
			Eigen::Vector2d point2 = Eigen::Vector2d(cos(particle.getState().z()), sin(particle.getState().z())) + point1;

			QLength minLength = std::numeric_limits<double>::quiet_NaN();

			for (const auto& line : walls) {

				Eigen::Vector2d point3 = line.first;
				Eigen::Vector2d point4 = line.second;

				// manually putting in to reduce overhead from creating Eigen matrix
				double t = (((point1.x() - point3.x()) * (point3.y() - point4.y())) - ((point1.y() - point3.y()) * (point3.x() - point4.x())))
				           / (((point1.x() - point2.x()) * (point3.y() - point4.y())) - ((point1.y() - point2.y()) * (point3.x() - point4.x())));

				double u = (((point1.x() - point2.y()) * (point1.y() - point3.y())) - ((point1.y() - point2.y()) * (point1.x() - point3.x())))
				           / (((point1.x() - point2.x()) * (point3.y() - point4.y())) - ((point1.y() - point2.y()) * (point3.x() - point4.x())));

				if (t >= 0 && 0 <= u && u <= 1) {
					if (isnan(minLength.getValue()) || minLength.getValue() > t) {
						minLength = t;
					}
				}
			}

			return minLength;
		}

		bool isOverLine(const Eigen::Vector2d& position) {
			return std::any_of(lines.begin(), lines.end(), [&](const auto &line) {
				return distanceToLine(line.first, line.second, position) < lineThickness/2.0;
			});
		}

		static QLength distanceToLine(Eigen::Vector2d point1, Eigen::Vector2d point2, Eigen::Vector2d point) {
			QLength distance1 = (point1 - point).norm();
			QLength distance2 = (point2 - point).norm();
			QLength distance = abs((point2.x() - point1.x()) * (point1.y() - point.y()) - (point1.x() - point.x()) * (point2.y() - point1.y())) / (point2 - point1).norm();

			return std::min(std::min(distance1, distance2), distance);
		}

        static Eigen::Vector3d fromGps(pros::c::gps_status_s gpsStatus) {
            return Eigen::Vector3d({gpsStatus.x, -gpsStatus.y, -gpsStatus.yaw * degree.Convert(radian)});
        }

		static Eigen::Vector2d minPosition;
		static Eigen::Vector2d maxPosition;
	};

	Eigen::Vector2d FieldModel::minPosition = Eigen::Vector2d(-1.8, -1.8);
	Eigen::Vector2d FieldModel::maxPosition = Eigen::Vector2d(1.8, 1.8);

    Particle::Particle(pros::c::gps_status_s gpsStatus) : Particle(FieldModel::fromGps(gpsStatus)) {
    }
}