#pragma once
#include "math.h"

namespace Kinemator {
	// Assuming no 'Z', convert pixel road segment to 2d road segment
	RoadSegment pixelTo2DRoadSegment(std::vector<PixelCoord> leftBoundary, std::vector<PixelCoord> rightBoundary,
		std::vector<PixelCoord> basisLine, double basisLength);

	TrackedObject flattenTrackedObject(TrackedObjectPixel pixelObj);


};