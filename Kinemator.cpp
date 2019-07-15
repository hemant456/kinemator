#include "Kinemator.h"

namespace Kinemator {

	std::vector<double> getTravels(const Rect& rectangle, const RoadSegment& road) {

		std::vector<double> travels;
		for (const auto& point : rectangle.points) {
			double speed = travelOnCenterLine(point, road);
			travels.push_back(speed);
		}

		std::sort(travels.begin(), travels.end());
	}

	std::vector<double> getSpeeds(const TrackedObject& object, const RoadSegment& road) {

		std::vector<double> speeds;
		const auto& footprints = object.footprints;

		for (int i = 1; i < footprints.size(); i++) {
			double dt = footprints[i].time - footprints[i - 1].time;
			const auto& lastTravels = getTravels(footprints[i - 1].shape, road);
			const auto& currentTravels = getTravels(footprints[i].shape, road);
			double speed = (currentTravels[0] - lastTravels[0]) / dt;
			speeds.push_back(speed);
		}

		return speeds;
	}

};