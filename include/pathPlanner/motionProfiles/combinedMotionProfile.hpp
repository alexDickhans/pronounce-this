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

		[[nodiscard]] MotionProfilePoint update(QTime t) const override {
			if (t > this->getDuration()) {
				return this->update(this->getDuration());
			}

			QTime totalTime = 0.0;
			QLength totalDistance = 0.0;
			double totalT = 0.0;

			for (int i = 0; i < motionProfiles.size(); i++) {
				QTime currentDuration = motionProfiles.at(i)->getDuration();
				if (totalTime + currentDuration >= t || motionProfiles.size() - 1 == i) {
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
		}

		[[nodiscard]] QTime getDuration() const override {
			QTime sum = 0.0;

			for (const auto &item: motionProfiles) {
				sum += item->getDuration();
			}

			return sum;
		}
	};
}