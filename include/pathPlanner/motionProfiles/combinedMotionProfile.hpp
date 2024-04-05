#pragma once

#include "abstractMotionProfile.hpp"
#include <numeric>

namespace PathPlanner {
	class CombinedMotionProfile : public AbstractMotionProfile {
	private:
		std::vector<std::shared_ptr<AbstractMotionProfile>> motionProfiles;
		QTime duration;
	public:
		explicit CombinedMotionProfile(const std::vector<std::shared_ptr<AbstractMotionProfile>> &motionProfile)
				: AbstractMotionProfile(), motionProfiles(motionProfile) {}

		CombinedMotionProfile() = default;

		void addMotionProfile(const std::shared_ptr<AbstractMotionProfile> &profile) {
			motionProfiles.emplace_back(profile);
			duration = 0.0;

			for (const auto &item : motionProfiles) {
				duration += isfinite(item->getDuration().getValue()) && item->getDuration().getValue() > 0.0 ? item->getDuration().getValue() : 0.0;
			}
		}

		[[nodiscard]] MotionProfilePoint update(const QTime time) const override {
			QTime t = std::min(time, this->getDuration());

			QTime totalTime = 0.0;
			QLength totalDistance = 0.0;
			double totalT = 0.0;

			for (const auto &profile: motionProfiles) {
				QTime currentDuration = profile->getDuration();
				if (totalTime + currentDuration >= t) {
					auto profiledPoint = profile->update(t - totalTime);

					profiledPoint.targetDistance += totalDistance;
					profiledPoint.targetT += totalT;

					return profiledPoint;
				}

				totalTime += currentDuration;
				auto profiledPoint = profile->update(totalTime);
				totalDistance += profiledPoint.targetDistance;
				totalT += profiledPoint.targetT;
			}

			auto profiledPoint = motionProfiles.at(motionProfiles.size()-1)->update(t - totalTime);

			profiledPoint.targetDistance += totalDistance;
			profiledPoint.targetT += totalT;

			return profiledPoint;
		}

		[[nodiscard]] QTime getDuration() const override {
			return duration;
		}
	};
}