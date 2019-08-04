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

std::vector<double> getSpeeds(const TrackedObject& object,
                              const RoadSegment& road) {

  std::vector<double> speeds;
  const auto& footprints = object.footprints;

  for (int i = 1; i < footprints.size(); i++) {
    double dt = footprints[i].time - footprints[i - 1].time;
    std::vector<double> distances;
    for (int j = 0; j < footprints[i].shape.points.size(); j++) {
      const double ds = distance_between_points(
          footprints[i].shape.points[j], footprints[i].shape.points[j - 1]);
      distances.push_back(ds);
    }
    // const auto& lastTravels = getTravels(footprints[i - 1].shape, road);
    // const auto& currentTravels = getTravels(footprints[i].shape, road);
    double speed = dt == 0 ? (distances[0]) * 1000 : (distances[0]) / dt;
    speeds.push_back(speed);
  }

  return speeds;
}

}; // namespace Kinemator