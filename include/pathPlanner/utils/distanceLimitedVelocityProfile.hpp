//
// Created by alex on 12/29/23.
//

#ifndef PRONOUNCE_THIS_DISTANCELIMITEDVELOCITYPROFILE_HPP
#define PRONOUNCE_THIS_DISTANCELIMITEDVELOCITYPROFILE_HPP

#include "Eigen/Dense"
#include <exception>

namespace PathPlanner {
	class DistanceLimitedTrapezoidalProfile {
	private:
		QLength distance;
		QLength startDistance;
		QTime time;
		QTime endTime;
		bool inverted;
		LinearInterpolator timeToDistance; // TODO: Better implementation for this
		LinearInterpolator timeToVelocity;
	public:
		DistanceLimitedTrapezoidalProfile() = delete;
		DistanceLimitedTrapezoidalProfile(std::vector<Eigen::Vector2d> velocityLimits, Eigen::Vector3d profileConstraints, bool inverted = false, QTime startTime = 0.0, QLength startDistance = 0.0, QSpeed startSpeed = 0.0, QSpeed endSpeed = 0.0) {
			if (velocityLimits.size() <= 2) {
				throw std::logic_error("Not enough velocity limits");
			}

			this->inverted = inverted;

			double lastLength = -10e100;
			this->startDistance = startDistance;

			std::for_each(velocityLimits.begin(), velocityLimits.end(), [&](auto &item) {
				if (lastLength >= item(0,0)) {
					throw std::logic_error("ERROR: last length bigger than current length: " + std::to_string(lastLength) + " > " + std::to_string(item(0, 0)));
				}
				lastLength = item(0, 0);
			});

			distance = velocityLimits[velocityLimits.size()-1](0, 0);

			QSpeed maxSpeed = profileConstraints(0, 0);
			QAcceleration maxAcceleration = profileConstraints(1, 0);
			QAcceleration maxDeceleration = std::max(profileConstraints(2, 0), maxAcceleration.getValue());

			auto maxSpeedLeft = std::vector<QSpeed>(velocityLimits.size());
			auto maxSpeedRight = std::vector<QSpeed>(velocityLimits.size());
			auto combinedMaxSpeed = std::vector<QSpeed>(velocityLimits.size());

			auto timeLeft = std::vector<QTime>(velocityLimits.size() - 1);
			auto timeRight = std::vector<QTime>(velocityLimits.size() - 1);
			auto combinedTime = std::vector<QTime>(velocityLimits.size() - 1);

			maxSpeedLeft[0] = startSpeed;
			maxSpeedRight[maxSpeedRight.size() - 1] = endSpeed;

			for (size_t i = 1; i < maxSpeedLeft.size(); i++) {
				QLength deltaDistance = velocityLimits[i](0, 0) - velocityLimits[i - 1](0, 0);
				QSpeed lastSpeed = maxSpeedLeft[i - 1];

				QSpeed currentMaxSpeed = Qsqrt(Qsq(lastSpeed) + 2 * maxAcceleration * deltaDistance);
				QSpeed limitedSpeed = std::min(std::min(currentMaxSpeed.getValue(), velocityLimits[i](1, 0)),
											   maxSpeed.getValue());

				maxSpeedLeft[i] = limitedSpeed;

				QTime duration =
						(2 * deltaDistance * (limitedSpeed - lastSpeed)) / (Qsq(limitedSpeed) - Qsq(lastSpeed));

				if (isnan(duration.getValue())) {
					duration = deltaDistance/limitedSpeed;
				}

				timeLeft.at(i - 1) = duration;
			}

			for (int i = maxSpeedRight.size()-2; i >= 0; i--) {
				QLength deltaDistance = velocityLimits[i + 1](0, 0) - velocityLimits[i](0, 0);
				QSpeed lastSpeed = maxSpeedRight[i + 1];

				QSpeed currentMaxSpeed = Qsqrt(Qsq(lastSpeed) + 2 * maxDeceleration * deltaDistance);
				QSpeed limitedSpeed = std::min(std::min(currentMaxSpeed.getValue(), velocityLimits[i](1, 0)),
											   maxSpeed.getValue());

				maxSpeedRight[i] = limitedSpeed;

				QTime duration =
						(2 * deltaDistance * (limitedSpeed - lastSpeed)) / (Qsq(limitedSpeed) - Qsq(lastSpeed));

				if (isnan(duration.getValue())) {
					duration = deltaDistance/limitedSpeed;
				}

				timeRight.at(i) = duration;
			}

			std::vector<QTime> timeIndex = std::vector<QTime>({startTime});

			for (size_t i = 0; i < combinedTime.size(); i++) {
				combinedTime[i] = std::max(timeLeft[i], timeRight[i]);
				time += combinedTime[i];
				timeIndex.emplace_back(time + startTime);
			}

			for (size_t i = 0; i < combinedMaxSpeed.size(); i++) {
				combinedMaxSpeed[i] = std::min(maxSpeedLeft[i], maxSpeedRight[i]);
			}

			for (size_t i = 0; i < timeIndex.size(); i++) {
				timeToVelocity.add(timeIndex[i].Convert(second), combinedMaxSpeed[i].getValue());
				timeToDistance.add(timeIndex[i].Convert(second), velocityLimits[i](0, 0));
			}

			endTime = time + startTime;
		}

		bool isReversed() const {
			return inverted;
		}

		QSpeed getSpeed(const QTime currentTime) {
			return timeToVelocity.get(currentTime.Convert(second)) * (inverted ? -1 : 1);
		}

		QLength getDistance(const QTime currentTime) {
			return timeToDistance.get(currentTime.Convert(second)) * (inverted ? -1 : 1) + startDistance.getValue();
		}

		QLength getRawDistance(const QTime currentTime) {
			return timeToDistance.get(currentTime.Convert(second));
		}

		QLength getLength() {
			return distance;
		}

		QLength getEndDistance() {
			return startDistance.getValue() + distance.getValue() * (inverted ? -1.0 : 1.0);
		}

		QTime getDuration() {
			return time;
		}

		QTime getEndTime() {
			return endTime;
		}
	};

} // PathPlanner

#endif //PRONOUNCE_THIS_DISTANCELIMITEDVELOCITYPROFILE_HPP
