#pragma once

#include "continuousOdometry.hpp"

namespace Pronounce {
	template<size_t size>
	class ParticleFilterOdometry : public ContinuousOdometry {
	private:
		Loco::ParticleFilter<size> &particleFilter;
	public:
		explicit ParticleFilterOdometry(Loco::ParticleFilter<size> &particleFilter) : particleFilter(particleFilter) {}

		void update() override {
			this->setPose(Loco::FieldModel::toPronounceRed(particleFilter.getMeanParticle().getState()));
		}

		~ParticleFilterOdometry() = default;
	};
}