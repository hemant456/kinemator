#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

namespace Kinemator {
struct PixelCoord {
  int x;
  int y;
  PixelCoord(int xIn, int yIn) {
    x = xIn;
    y = yIn;
  }
};

struct Point2D {
  double x;
  double y;

  Point2D() { x = y = 0.0; }
  Point2D(double x_in, double y_in) {
    x = x_in;
    y = y_in;
  }

  Point2D operator+(const Point2D& another) const {
    return Point2D(x + another.x, y + another.y);
  }

  Point2D operator*(const double factor) const {
    return Point2D(x * factor, y * factor);
  }

  Point2D stretch(const double margin) const {
    return Point2D(x * margin, y * margin);
  }

  Point2D rotated(double radians) const // + ve if counterclock wise
  {
    return Point2D(x * cos(radians) - y * sin(radians),
                   y * cos(radians) + x * sin(radians));
  }

  std::string str() const {
    std::stringstream ss;
    ss << "(" << x << ", " << y << ")";
    return ss.str();
  }
}; // namespace Kinemator

struct Line {
  Point2D p;
  Point2D q;

  Line(Point2D p_in, Point2D q_in) {
    p = p_in;
    q = q_in;
  }

  Point2D project(const Point2D& toProject) const;

  bool intersectionPoint(const Line& another, Point2D& isecPoint) const;
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

  Footprint flatten(const cv::Mat& mat) const {

    std::vector<cv::Point2f> worldPoints;
    std::vector<cv::Point2f> cameraPoints;

    for (const auto& pt : shape) {
      cameraPoints.push_back(cv::Point2f(pt.x, pt.y));
    }
    perspectiveTransform(cameraPoints, worldPoints, mat);

    std::vector<Point2D> flatShape;
    for (const auto& pt : worldPoints) {
      flatShape.push_back(Point2D(pt.x, pt.y));
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
  cv::Mat mat;
  Point2D topLeft, topRight, bottomLeft, bottomRight;
  double longDistance; // this is distance between far end and near end of the
                       // road.
  double
      latDistance; // distance between left and right boundary of the road (this
                   // should be around 3/3.2 meters and match lane width)
  std::vector<CrossTrackBand> orthogonalBands;
  std::vector<double> bandsTravel;
};

inline double distance_between_points(const Point2D& a, const Point2D& b) {
  return sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
}

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(const Point2D& p, const Point2D& q, const Point2D& r);

double travelOnCenterLine(const Point2D& point, const RoadSegment& road);
}; // namespace Kinemator
