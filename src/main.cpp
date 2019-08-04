// Kinemator.cpp : This file contains the 'main' function. Program execution
// begins and ends there.
//

#include "ImagePlanner.h"
#include "Kinemator.h"
#include "kinemator_math.h"
#include <iostream>
#include <unordered_map>
#include <vector>

namespace km = Kinemator;
bool runTest(std::vector<km::PixelCoord> leftBoundary,
             std::vector<km::PixelCoord> rightBoundary,
             std::vector<km::PixelCoord> basisCoords, double basisLength,
             double basisWidth,
             std::vector<km::TrackedObjectPixel> pixelwiseTrackedObjects) {
  km::RoadSegment road = pixelTo2DRoadSegment(
      leftBoundary, rightBoundary, basisCoords, basisLength, basisWidth);

  std::cout << road.bottomLeft.str() << " " << road.topLeft.str() << "\n";
  std::cout << road.bottomRight.str() << " " << road.topRight.str() << "\n";

  std::vector<km::TrackedObject> trackedObjects;
  for (const auto& object : pixelwiseTrackedObjects) {
    trackedObjects.push_back(km::flattenTrackedObject(object, road.mat));
  }

  // call flattenTrackedObject for all tracked objects
  // call Kinemator get speeds
  std::unordered_map<int, std::vector<double>> trackedSpeeds;
  for (const auto& object : trackedObjects) {
    const std::vector<double>& speeds = km::getSpeeds(object, road);
    trackedSpeeds[object.id] = speeds;
  }

  for (auto ele : trackedSpeeds) {
    std::cout << ele.first << "#";
    for (auto speed : ele.second) {
      std::cout << speed << ", ";
    }
    std::cout << " m/s";
  }
  return true;
}
int main() {
  // call pixelTo2DRoadSegment
  std::vector<Kinemator::PixelCoord> leftBoundary, rightBoundary;
  std::vector<Kinemator::PixelCoord> basisCoords;
  double basisLength = 1; // meters
  double basisWidth = 1;  // meters

  basisCoords = {
      Kinemator::PixelCoord(608, 2746), Kinemator::PixelCoord(700, 2248),
      Kinemator::PixelCoord(1376, 2260), Kinemator::PixelCoord(1394, 2758)};

  leftBoundary = {Kinemator::PixelCoord(1376, 2260),
                  Kinemator::PixelCoord(1345, 829)};
  rightBoundary = {Kinemator::PixelCoord(2073, 2279),
                   Kinemator::PixelCoord(1726, 841)};

  std::vector<km::PixelCoord> shape0 = {
      km::PixelCoord(866, 1333), km::PixelCoord(915, 1142),
      km::PixelCoord(1345, 1148), km::PixelCoord(1357, 1339)};
  std::vector<km::PixelCoord> shape1 = {
      km::PixelCoord(866, 1333), km::PixelCoord(915, 1142),
      km::PixelCoord(1345, 1148), km::PixelCoord(1357, 1339)};
  std::vector<km::PixelCoord> shape2 = {
      km::PixelCoord(7706, 2258), km::PixelCoord(768, 1873),
      km::PixelCoord(1370, 1892), km::PixelCoord(1370, 2267)};
  std::vector<km::PixelCoord> shape3 = {
      km::PixelCoord(460, 3422), km::PixelCoord(602, 2746),
      km::PixelCoord(1400, 2758), km::PixelCoord(1406, 3471)};

  km::PixelFootprint pixelfp0{.shape = shape0, .time = 1};
  km::PixelFootprint pixelfp1{.shape = shape1, .time = 3};
  km::PixelFootprint pixelfp2{.shape = shape2, .time = 5};
  km::PixelFootprint pixelfp3{.shape = shape3, .time = 7};

  km::TrackedObjectPixel pixelTrackedObj;
  pixelTrackedObj.footprints = {pixelfp0, pixelfp1, pixelfp2, pixelfp3};
  pixelTrackedObj.id = 101;

  runTest(leftBoundary, rightBoundary, basisCoords, basisLength, basisWidth,
          {pixelTrackedObj});

  // HAVE FUN :-) trackedSpeeds is a map of tracked ID to track speeds (speeds
  // of all four corners of track rectangle)

  std::cout << "Hello World!\n";
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started:
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add
//   Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project
//   and select the .sln file
