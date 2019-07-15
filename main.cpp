// Kinemator.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <unordered_map>
#include "Kinemator.h"
#include "math.h"
#include "ImagePlaner.h"

int main()
{
	 // call pixelTo2DRoadSegment
	std::vector<Kinemator::PixelCoord> leftBoundary, rightBoundary;
	std::vector<Kinemator::PixelCoord> basisLine; double basisLength;

	Kinemator::RoadSegment road = pixelTo2DRoadSegment(leftBoundary, rightBoundary, basisLine, basisLength);

	std::vector<Kinemator::TrackedObjectPixel> pixelwiseTrackedObjects;
	std::vector<Kinemator::TrackedObject> trackedObjects;
	for (const auto& object : pixelwiseTrackedObjects) {
		trackedObjects.push_back(Kinemator::flattenTrackedObject(object));
	}

	// call flattenTrackedObject for all tracked objects
	// call Kinemator get speeds 
	std::unordered_map<int, std::vector<double>> trackedSpeeds;
	for (const auto& object : trackedObjects) {
		const std::vector<double>& speeds = Kinemator::getSpeeds(object, road);
		trackedSpeeds[object.id] = speeds;
	}
	
	// HAVE FUN :-) trackedSpeeds is a map of tracked ID to track speeds (speeds of all four corners of track rectangle)

	std::cout << "Hello World!\n";
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
