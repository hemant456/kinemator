#include "..\Kinemator\math.h"

namespace Kinemator {
	double travelOnCenterLine(const Point2D& point, const RoadSegment& road) {
		// Step 1: Project given point on the road segment's centerline.
		// Step 2: Interpolate the long distance.

		double minPerpDistance = 10000;
		double longDist = 0;
		Point2D perpPt;

		for (int i = 0; i < road.orthogonalBands.size(); i++) {
			const auto& band = road.orthogonalBands[i];
			Point2D vec1(band.topCenter.x - band.bottomCenter.x, band.topCenter.y - band.bottomCenter.y);
			Point2D ptX = vec1.perpPoint(point);
			if (onsegment(band.bottomCenter, ptX, band.topCenter)) {
				double travel = distance_between_points(ptX, band.bottomCenter);
				perpPt = ptX;
				longDist += travel;
				break;
			}
			longDist += road.bandsTravel[i];
		}

		return longDist;
	}
}