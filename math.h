#pragma once
#include <vector>
#include <algorithm>

namespace Kinemator {
	struct PixelCoord {
		int x;
		int y;
	};

	struct Point2D {
		double x;
		double y;

		Point2D() { x = y = 0.0; }
		Point2D(double x_in, double y_in) { x = x_in; y = y_in; }

		Point2D operator+(const Point2D& another) const {
			return Point2D(x + another.x, y + another.y);
		}

		Point2D operator*(const double factor) const {
			return Point2D(x * factor, y * factor);
		}

		Point2D stretch(const double margin) const {
			return Point2D(x * margin, y * margin);
		}
	};

	struct Rect {
		std::vector<Point2D> points;
	};

	struct Footprint {
		Rect shape;
		double time; // time in seconds.
	};

	struct PixelFootprint {
		std::vector<PixelCoord> shape;
		double time; // time in seconds.

		Footprint flatten() const {
			std::vector<Point2D> flatShape;
			for (const auto& pixelCoord : shape) {
				flatShape.push_back(Point2D(pixelCoord.x, pixelCoord.y));
			}

			Footprint flat;
			flat.shape.points = flatShape;
			flat.time = time;
			return flat;
		}
	};

	struct TrackedObjectPixel {
		int id;
		std::vector<PixelFootprint> footprints;
	};

	struct TrackedObject {
		int id;
		std::vector<Footprint> footprints;
	};

	struct CrossTrackBand {
		Point2D topLeft, topRight, topCenter, bottomLeft, bottomRight, bottomCenter;
	};

	struct RoadSegment {
		Point2D topLeft, topRight, bottomLeft, bottomRight;
		double longDistance; // this is distance between far end and near end of the road.
		double latDistance; // distance between left and right boundary of the road (this should be around 3/3.2 meters and match lane width)
		std::vector<CrossTrackBand> orthogonalBands;
		std::vector<double> bandsTravel;
	};

	double distance_between_points(const Point2D& a, const Point2D& b) {
		return sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
	}

	bool onsegment(const Point2D& a, const Point2D& b, const Point2D& c) {
		return ((a.x <= b.x && b.x <= c.x) || (a.x >= b.x && b.x >= c.x)) && ((a.y <= b.y && b.y <= c.y) || (a.y >= b.y && b.y >= c.y));
	}
	double travelOnCenterLine(const Point2D& point, const RoadSegment& road);

};
