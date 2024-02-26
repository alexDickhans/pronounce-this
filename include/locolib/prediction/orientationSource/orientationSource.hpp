#pragma once

namespace Loco {
	class OrientationSource {
	public:
		OrientationSource() = default;

		virtual Angle getAngle() {
			return 0.0;
		}
	};
}