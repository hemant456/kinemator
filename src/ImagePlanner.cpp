#include "ImagePlanner.h"
#include "kinemator_math.h"
#include <assert.h>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace cv;
namespace Kinemator {

TrackedObject flattenTrackedObject(TrackedObjectPixel pixelObj,
                                   const cv::Mat& mat) {
  TrackedObject object;
  object.id = pixelObj.id;

  for (const auto& pixelFootprint : pixelObj.footprints) {

    object.footprints.push_back(pixelFootprint.flatten(mat));
  }

  return object;
}

// Assuming no 'Z', convert pixel road segment to 2d road segment
RoadSegment pixelTo2DRoadSegment(std::vector<PixelCoord> leftBoundary,
                                 std::vector<PixelCoord> rightBoundary,
                                 std::vector<PixelCoord> basisCoords,
                                 double basisLength, double basisWidth) {

  // Input Quadilateral or Image plane coordinates
  std::vector<Point2f> worldPoints;
  // Output Quadilateral or World plane coordinates
  std::vector<Point2f> cameraPoints;

  worldPoints.resize(4);
  cameraPoints.resize(4);

  // The 4 points that select quadilateral on the input , from top-left in
  // clockwise order These four pts are the sides of the rect box used as input
  worldPoints[0] = Point2f(0, 0);
  worldPoints[1] = Point2f(0, basisLength);
  worldPoints[2] = Point2f(basisWidth, basisLength);
  worldPoints[3] = Point2f(basisWidth, 0);

  // The 4 points where the mapping is to be done , from top-left in clockwise
  // order
  cameraPoints[0] = Point2f(basisCoords[0].x, basisCoords[0].y);
  cameraPoints[1] = Point2f(basisCoords[1].x, basisCoords[1].y);
  cameraPoints[2] = Point2f(basisCoords[2].x, basisCoords[2].y);
  cameraPoints[3] = Point2f(basisCoords[3].x, basisCoords[3].y);

  Mat perspectiveMat_ = findHomography(cameraPoints, worldPoints, CV_RANSAC);

  std::vector<Point2D> leftBoundary2D, rightBoundary2D;

  std::vector<Point2f> cameraCorners;
  std::vector<Point2f> worldCorners;

  for (const auto& pt : leftBoundary) {
    cameraCorners.push_back(Point2f(pt.x, pt.y));
  }

  perspectiveTransform(cameraCorners, worldCorners, perspectiveMat_);

  for (const auto& pt : worldCorners) {
    leftBoundary2D.push_back(Point2D(pt.x, pt.y));
  }

  cameraCorners.clear();
  for (const auto& pt : rightBoundary) {
    cameraCorners.push_back(Point2f(pt.x, pt.y));
  }

  perspectiveTransform(cameraCorners, worldCorners, perspectiveMat_);

  for (const auto& pt : worldCorners) {
    rightBoundary2D.push_back(Point2D(pt.x, pt.y));
  }

  // Generate road structure (For now, it is just a quad, but road can be
  // divided in one meter segments for better resolution for curving roads etc.
  // Rest of the code works with the road segments, and does not care about the
  // curves, turns, or resolution. So better resolution should automatically
  // yield better math later)
  double totalLongTravel = 0;
  RoadSegment road;
  for (int i = 0; i < leftBoundary2D.size() - 1; i++) {
    CrossTrackBand band;
    band.bottomLeft = leftBoundary2D[i];
    band.bottomRight = rightBoundary2D[i];
    band.bottomCenter = (band.bottomLeft + band.bottomRight) * 0.5;

    band.topLeft = leftBoundary2D[i + 1];
    band.topRight = rightBoundary2D[i + 1];
    band.topCenter = (band.topLeft + band.topRight) * 0.5;

    road.orthogonalBands.push_back(band);
    const double travel =
        distance_between_points(band.bottomCenter, band.topCenter);
    road.bandsTravel.push_back(travel);
    totalLongTravel += travel;
  }

  road.topLeft = road.orthogonalBands.back().topLeft;
  road.topRight = road.orthogonalBands.back().topRight;
  road.bottomLeft = road.orthogonalBands.front().bottomLeft;
  road.bottomRight = road.orthogonalBands.front().bottomRight;
  road.longDistance = totalLongTravel;

  road.mat = perspectiveMat_;
  return road;
}

}; // namespace Kinemator