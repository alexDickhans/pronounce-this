#pragma once

#include "abstractMotionProfile.hpp"
#include <numeric>

namespace PathPlanner {
	class CombinedMotionProfile : public AbstractMotionProfile {
	private:
		std::vector<std::shared_ptr<AbstractMotionProfile>> motionProfiles;
	public:
		explicit CombinedMotionProfile(const std::vector<std::shared_ptr<AbstractMotionProfile>> &motionProfile)
				: AbstractMotionProfile(), motionProfiles(motionProfile) {}

		CombinedMotionProfile() = default;

		void addMotionProfile(const std::shared_ptr<AbstractMotionProfile> &profile) {
			motionProfiles.emplace_back(profile);
		}

		[[nodiscard]] MotionProfilePoint update(const QTime time) const override {
			QTime t = std::min(time, this->getDuration());

			QTime totalTime = 0.0;
			QLength totalDistance = 0.0;
			double totalT = 0.0;

			for (int i = 0; i < motionProfiles.size()-1; i++) {
				QTime currentDuration = motionProfiles.at(i)->getDuration();
				if (totalTime + currentDuration >= t) {
					auto profiledPoint = motionProfiles.at(i)->update(t - totalTime);

					profiledPoint.targetDistance += totalDistance;
					profiledPoint.targetT += totalT;

					return profiledPoint;
				}

				totalTime += currentDuration;
				auto profiledPoint = motionProfiles.at(i)->update(totalTime);
				totalDistance += profiledPoint.targetDistance;
				totalT += profiledPoint.targetT;
			}

			auto profiledPoint = motionProfiles.at(motionProfiles.size()-1)->update(t - totalTime);

			profiledPoint.targetDistance += totalDistance;
			profiledPoint.targetT += totalT;

			return profiledPoint;
		}

		[[nodiscard]] QTime getDuration() const override {
			QTime sum = 0.0;

			for (const auto &item: motionProfiles) {
				sum += isnan(item->getDuration().getValue()) ? 0.0 : item->getDuration().getValue();
			}

			return sum;
		}
	};
}