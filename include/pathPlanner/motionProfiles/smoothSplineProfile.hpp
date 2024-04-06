#pragma once

#include "abstractMotionProfile.hpp"
#include "combinedMotionProfile.hpp"
#include "pathPlanner/utils/bezierSegment.hpp"

namespace PathPlanner {

	struct MaxSpeedPoint_ {
		QLength distance;
		QVelocity velocity;
		QAcceleration accel;
	} typedef MaxSpeedPoint;

	class SmoothSplineProfile : public AbstractMotionProfile {
	private:
		std::vector<BezierSegment> bezierSegment;

		LinearInterpolator timeToVelocityInterpolator;

		QTime duration = 0.0;

		QVelocity startSpeed = 0.0;
		QVelocity endSpeed = 0.0;

		static std::vector<MaxSpeedPoint>
		limitSpeed(const std::vector<MaxSpeedPoint> &unlimitedSpeed, QVelocity startSpeed) {
			std::vector<MaxSpeedPoint> result = unlimitedSpeed;
			result.at(0).velocity = startSpeed;

			for (auto item = result.begin() + 1; item < result.end(); item++) {
				QLength deltaDistance = Qabs(item->distance - (item - 1)->distance);
				QVelocity endVelocity = std::min(
						Qsqrt((Qsq((item - 1)->velocity) + 2 * (item - 1)->accel * deltaDistance)), item->velocity);
				item->velocity = endVelocity;
				(item - 1)->accel = (Qsq(endVelocity) - Qsq((item - 1)->velocity)) / (2 * deltaDistance);
			}

			return result;
		}

	protected:
		void calculate(QLength gran = 1_in) {
			QLength totalDistance = 0.0;

			std::vector<MaxSpeedPoint> maxSpeedArray;//({{0.0, 0.0, 0.0}});

			for (const auto &segment: bezierSegment) {
				QLength distance = segment.getDistance();

				int granularity = std::max(5, static_cast<int>(distance.Convert(gran)));
				QLength intervalDistance = distance / granularity;

				if (!maxSpeedArray.empty()) {
					maxSpeedArray.at(maxSpeedArray.size() - 1).velocity =
							min(maxSpeedArray.at(maxSpeedArray.size() - 1).velocity,
							    segment.getMaxSpeedMultiplier(Constants::trackWidth, 0.0) *
							    segment.getProfileConstraints().maxVelocity);
				}

				for (int i = 1; i <= granularity; i++) {
					maxSpeedArray.emplace_back(totalDistance + (i * intervalDistance),
					                           segment.getMaxSpeedMultiplier(Constants::trackWidth, segment.getTByLength(
							                           i * intervalDistance)) *
					                           segment.getProfileConstraints().maxVelocity,
					                           segment.getProfileConstraints().maxAcceleration);
				}

				totalDistance += distance;
			}

			// Do left and right passes
			std::reverse(maxSpeedArray.begin(), maxSpeedArray.end());
			auto rightPassMin = PathPlanner::SmoothSplineProfile::limitSpeed(maxSpeedArray, endSpeed);
			std::reverse(rightPassMin.begin(), rightPassMin.end());
			std::reverse(maxSpeedArray.begin(), maxSpeedArray.end());
			auto leftPassMin = PathPlanner::SmoothSplineProfile::limitSpeed(maxSpeedArray, startSpeed);

			for (int i = 0; i < maxSpeedArray.size(); i++) {
				maxSpeedArray.at(i).velocity = std::min(leftPassMin.at(i).velocity, rightPassMin.at(i).velocity);
			}

			duration = 0.0;

			timeToVelocityInterpolator.add(duration.getValue(), maxSpeedArray.at(0).velocity.getValue());

			// Calculate time for finalized array
			for (int i = 1; i < maxSpeedArray.size(); i++) {
				// add stuff to current time
				QLength deltaDistance = maxSpeedArray.at(i).distance - maxSpeedArray.at(i - 1).distance;
				QAcceleration a =
						(Qsq(maxSpeedArray.at(i).velocity) - Qsq(maxSpeedArray.at(i-1).velocity)) / (2 * deltaDistance);

				if (abs(a.getValue()) > 0.0001) {
					duration += (maxSpeedArray.at(i).velocity - maxSpeedArray.at(i - 1).velocity) / a;
				} else {
					duration += deltaDistance / maxSpeedArray.at(i).velocity;
				}

				timeToVelocityInterpolator.add(duration.getValue(), maxSpeedArray.at(i).velocity.getValue());
			}
		}

		[[nodiscard]] std::pair<size_t, double> getTAtDistance(QLength distance) const {

			QLength distanceRemaining = Qabs(distance);

			for (int i = 0; i < bezierSegment.size(); i++) {
				auto segmentLength = Qabs(bezierSegment.at(i).getDistance());
				if (distanceRemaining <= segmentLength) {
					return {i, bezierSegment.at(i).getTByLength(distanceRemaining)};
				}
				distanceRemaining -= segmentLength;
			}

			return {bezierSegment.size() - 1, 1};
		}

	public:

		explicit SmoothSplineProfile(const std::vector<BezierSegment> &bezierSegment)
				: AbstractMotionProfile(), bezierSegment(bezierSegment) {
			this->calculate();
		}

		static std::shared_ptr<CombinedMotionProfile> build(const std::vector<BezierSegment> &bezierSegment) {
			auto motionProfile = std::make_shared<CombinedMotionProfile>();

			if (bezierSegment.empty()) {
				return motionProfile;
			}

			std::vector<BezierSegment> currentSplines = {bezierSegment.at(0)};

			for (int i = 1; i < bezierSegment.size(); i++) {
				if (bezierSegment.at(i - 1).isStopEnd() ||
				    bezierSegment.at(i - 1).isReversed() != bezierSegment.at(i).isReversed()) {
					motionProfile->addMotionProfile(std::make_shared<SmoothSplineProfile>(currentSplines));
					currentSplines.clear();
				}

				currentSplines.emplace_back(bezierSegment.at(i));
			}

			motionProfile->addMotionProfile(std::make_shared<SmoothSplineProfile>(currentSplines));

			return motionProfile;
		}

		static std::shared_ptr<CombinedMotionProfile> build(asset path) {
			Json parsed_path = open_asset_as_json(path);
			std::vector<BezierSegment> bezierSegment;

			auto jsonPathSegments = parsed_path["segments"];

			std::for_each(jsonPathSegments.array_items().begin(), jsonPathSegments.array_items().end(),
			              [&](const auto &segment) {
				              auto constraints = segment["constraints"];
				              auto paths = segment["paths"];
				              bezierSegment.emplace_back(BezierSegment(paths[0], paths[1], paths[2], paths[3],
				                                                             segment["inverted"].bool_value(),
				                                                             segment["stopEnd"].bool_value(),
				                                                             {constraints["velocity"].number_value() * inch/second,
				                                                              constraints["accel"].number_value() * inch/second/second,
				                                                              0.0}));
			              });

			auto noCommands = build(bezierSegment);

			noCommands->processCommands(parsed_path["commands"]);

			return noCommands;
		}

		[[nodiscard]] MotionProfilePoint update(const QTime time) const override {

			QTime t = std::min(duration, time);

			MotionProfilePoint point;

			double invertedMultiplier = this->bezierSegment.at(0).isReversed() ? -1 : 1;

			point.targetDistance = timeToVelocityInterpolator.getIntegral(t.getValue()) * invertedMultiplier;
			auto [index, targetT] = getTAtDistance(Qabs(point.targetDistance));
			point.targetT = static_cast<double>(index) + targetT;
			point.targetSpeed = timeToVelocityInterpolator.get(t.getValue()) * invertedMultiplier;
			auto targetPoint = bezierSegment.at(index).evaluate(targetT);
			point.targetPose = Pronounce::Pose2D(targetPoint.getX(), targetPoint.getY(),
			                                     bezierSegment.at(index).getAngle(targetT));
			point.targetCurvature = bezierSegment.at(index).getCurvature(targetT);

			Log(string_format("Time: %f, Distance: %f, Speed: %f, Angle: %f, Curvature: %f, Index: %d, T: %f", time.Convert(second), point.targetDistance.Convert(inch), point.targetSpeed.Convert(inch/second), point.targetPose.getAngle().Convert(degree), point.targetCurvature.Convert(degree/inch), index, targetT));

			return point;
		}

		[[nodiscard]] QTime getDuration() const override {
			return duration;
		}
	};

} // Pronounce

#define SMOOTH_SPLINE_PATH_ASSET(x) ASSET(x##_json); auto x = PathPlanner::SmoothSplineProfile::build(x##_json);
