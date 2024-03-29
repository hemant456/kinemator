#pragma once
#include "kinemator_math.h"

namespace Kinemator {

std::vector<double> getTravels(const Rect& rectangle, const RoadSegment& road);
std::vector<double> getSpeeds(const TrackedObject& object,
                              const RoadSegment& road);

} // namespace Kinemator
