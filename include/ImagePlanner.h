#pragma once
#include "kinemator_math.h"

namespace Kinemator {
// Assuming no 'Z', convert pixel road segment to 2d road segment
RoadSegment pixelTo2DRoadSegment(std::vector<PixelCoord> leftBoundary,
                                 std::vector<PixelCoord> rightBoundary,
                                 std::vector<PixelCoord> basisCoords,
                                 double basisLength, double basisWidth);

TrackedObject flattenTrackedObject(TrackedObjectPixel pixelObj,
                                   const cv::Mat& mat);

}; // namespace Kinemator