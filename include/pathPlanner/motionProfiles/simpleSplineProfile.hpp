#pragma once

#include "abstractMotionProfile.hpp"
namespace PathPlanner {
	class SimpleSplineProfile : public AbstractMotionProfile {
		std::vector<std::pair<BezierSegment, std::shared_ptr<Pronounce::VelocityProfile>>> pathSegments;
	public:
		SimpleSplineProfile(const Pronounce::ProfileConstraints &defaultProfileConstraints,
		                    const std::vector<std::pair<BezierSegment, std::shared_ptr<Pronounce::VelocityProfile>>> &pathSegments)
				: pathSegments(pathSegments) { calculate(defaultProfileConstraints); }

		explicit SimpleSplineProfile(asset path) {
			Json parsed_path = open_asset_as_json(path);

			std::vector<std::pair<BezierSegment, Pronounce::VelocityProfile*>> parsedPath;

			auto segments = parsed_path["segments"];

			for (const auto& segment : segments.array_items()) {
				auto constraints = segment["constraints"];
				auto paths = segment["paths"];

				parsedPath.emplace_back(
						BezierSegment(
								Point(paths[0]),
								Point(paths[1]),
								Point(paths[2]),
								Point(paths[3]),
								segment["inverted"].bool_value(),
								segment["stopEnd"].bool_value()),
						new Pronounce::SinusoidalVelocityProfile(0.0,
						                                         Pronounce::ProfileConstraints{
								                                         constraints["velocity"].number_value() * (inch/second).Convert(metre/second),
								                                         constraints["accel"].number_value() * (inch/second/second).Convert(metre/second/second),
								                                         0.0
						                                         }));
			}

			this->processCommands(parsed_path["commands"]);

			calculate();
		}

		SimpleSplineProfile(Pronounce::ProfileConstraints defaultProfileConstraints, std::vector<std::pair<BezierSegment, std::shared_ptr<Pronounce::VelocityProfile>>> path, std::initializer_list<std::pair<double, std::string>> functions = {}) {
			pathSegments = std::move(path);
			this->commands = functions;

			calculate(defaultProfileConstraints);
		}

		void calculate(Pronounce::ProfileConstraints defaultProfileConstraints = {0.0, 0.0, 0.0}) {
			for (int i = 0; i < this->pathSegments.size(); ++i) {

				QVelocity adjustedSpeed = this->pathSegments.at(i).first.getMaxSpeedMultiplier(trackWidth) * maxSpeed;

				auto profile = this->pathSegments.at(i).second;

				if (profile == nullptr) {
					profile = std::make_shared<Pronounce::SinusoidalVelocityProfile>(this->pathSegments.at(i).first.getDistance(), defaultProfileConstraints);
				}

				profile->setDistance(this->pathSegments.at(i).first.getDistance());

				profile->setProfileConstraints({std::min(profile->getProfileConstraints().maxVelocity, adjustedSpeed), profile->getProfileConstraints().maxAcceleration, 0.0});

				profile->setInitialSpeed(0.0);
				profile->setEndSpeed(0.0);

				if (i < this->pathSegments.size()-1) {
					if (this->pathSegments.at(i+1).first.isReversed() == this->pathSegments.at(i).first.isReversed() && !this->pathSegments.at(i).first.isStopEnd())
						profile->setEndSpeed(profile->getProfileConstraints().maxVelocity.getValue());// * (this->pathSegments.at(i).first.getReversed() ? -1.0 : 1.0));
				}

				if (i > 0) {
					if (this->pathSegments.at(i-1).first.isReversed() == this->pathSegments.at(i).first.isReversed() && !this->pathSegments.at(i-1).first.isStopEnd())
						profile->setInitialSpeed(this->pathSegments.at(i-1).second->getProfileConstraints().maxVelocity.getValue());// * (this->pathSegments.at(i).first.getReversed() ? -1.0 : 1.0));
				}

				profile->calculate();

				this->pathSegments.at(i).second = profile;
			}
		}

		[[nodiscard]] MotionProfilePoint update(QTime t) const override {
			MotionProfilePoint point;

			point.targetDistance = this->getDistance(t);

			QTime time = {t.getValue()};
			int index = 0;
			QLength totalDistance = 0.0;

			while (time >= pathSegments.at(index).second->getDuration()) {
				time -= pathSegments.at(index).second->getDuration();
				totalDistance += pathSegments.at(index).first.getDistance();
				index ++;
			}

			point.targetCurvature = pathSegments.at(index).first.getCurvature(pathSegments.at(index).first.getTByLength(abs(pathSegments.at(index).second->getDistanceByTime(time).getValue())));
			point.targetSpeed = pathSegments.at(index).second->getVelocityByTime(time);
			point.targetT = index + pathSegments.at(index).first.getTByLength(abs(pathSegments.at(index).second->getDistanceByTime(time).getValue()));
			Angle targetAngle = pathSegments.at(index).first.getAngle(abs(pathSegments.at(index).first.getTByLength(abs(pathSegments.at(index).second->getDistanceByTime(time).getValue()))));
			Point targetPoint = pathSegments.at(index).first.evaluate(abs(pathSegments.at(index).first.getTByLength(abs(pathSegments.at(index).second->getDistanceByTime(time).getValue()))));
			point.targetPose = {targetPoint.getX(), targetPoint.getY(), targetAngle};

			return point;
		}

		[[nodiscard]] QTime getDuration() const override {
			QTime totalTime = 0.0;

			std::for_each(pathSegments.begin(), pathSegments.end(), [&](const auto &item) {
				totalTime += item.second->getDuration();
			});

			Log(std::to_string(totalTime.getValue()));

			return totalTime;
		}

		[[nodiscard]] QLength getDistance(QTime time) const {
			int index = 0;
			QLength totalDistance = 0.0;

			while (time >= pathSegments.at(index).second->getDuration()) {
				time -= pathSegments.at(index).second->getDuration();
				totalDistance += pathSegments.at(index).first.getDistance();
				index ++;
			}

			QLength distance = pathSegments.at(index).second->getDistanceByTime(time) + totalDistance;

			return distance;
		}

	};
}

#define SIMPLE_SPLINE_PATH_ASSET(x) ASSET(x##_json); auto x = std::make_shared<PathPlanner::SimpleSplineProfile>(x##_json);
