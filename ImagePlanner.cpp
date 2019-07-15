#include "ImagePlaner.h"
#include "math.h"

namespace Kinemator {

	TrackedObject flattenTrackedObject(TrackedObjectPixel pixelObj) {
		TrackedObject object;
		object.id = pixelObj.id;

		for (const auto& pixelFootprint : pixelObj.footprints) {
			object.footprints.push_back(pixelFootprint.flatten());
		}

		return object;
	}

	// Assuming no 'Z', convert pixel road segment to 2d road segment
	RoadSegment pixelTo2DRoadSegment(std::vector<PixelCoord> leftBoundary, std::vector<PixelCoord> rightBoundary,
		std::vector<PixelCoord> basisLine, double basisLength) {

		std::vector<Point2D> leftBoundary2D, rightBoundary2D;

		for (int i = 0; i <= basisLine.size(); i++) {
			PixelCoord basisLinePt = basisLine[i];
			Point2D vec(basisLinePt.x, basisLinePt.y);
			vec.rotate(M_PI / 2);
			Point2D leftExtremePt = vec.stretch(1000);
			Point2D rightExtremePt = leftExtremePt.rotate(M_PI);
			Point2D leftRoadPt = IntersectPixelBoundary(leftBoundary, Line(leftExtremePt, rightExtremePt));
			Point2D rightRoadPt = IntersectPixelBoundary(rightBoundary, Line(leftExtremePt, rightExtremePt));

			leftBoundary2D.push_back(leftRoadPt);
			rightBoundary2D.push_back(rightRoadPt);
		}

		// Generate road structure
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
			const double travel = basisLength / (basisLine.size() - 1);
			road.bandsTravel.push_back(travel);
			totalLongTravel += travel;
		}

		road.topLeft = road.orthogonalBands.back().topLeft;
		road.topRight = road.orthogonalBands.back().topRight;
		road.bottomLeft = road.orthogonalBands.front().bottomLeft;
		road.bottomRight = road.orthogonalBands.front().bottomRight;
		road.longDistance = totalLongTravel;

		return road;
	}


};