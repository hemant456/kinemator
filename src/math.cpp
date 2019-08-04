#include "kinemator_math.h"

namespace Kinemator {
bool onSegment(const Point2D& p, const Point2D& q, const Point2D& r) {
  if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
      q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
    return true;

  return false;
}

double travelOnCenterLine(const Point2D& point, const RoadSegment& road) {
  // Step 1: Project given point on the road segment's centerline.
  // Step 2: Interpolate the long distance.

  double minPerpDistance = 10000;
  double longDist = 0;
  Point2D perpPt;

  for (int i = 0; i < road.orthogonalBands.size(); i++) {
    const auto& band = road.orthogonalBands[i];
    Point2D vec1(band.topCenter.x - band.bottomCenter.x,
                 band.topCenter.y - band.bottomCenter.y);
    Point2D ptX = Line(band.bottomCenter, band.topCenter).project(point);
    if (onSegment(band.bottomCenter, ptX, band.topCenter)) {
      double travel = distance_between_points(ptX, band.bottomCenter);
      perpPt = ptX;
      longDist += travel;
      break;
    }
    longDist += road.bandsTravel[i];
  }

  return longDist;
}

Point2D Line::project(const Point2D& toProject) const {

  if (q.x == p.x) {
    return Point2D(p.x, toProject.y);
  }

  const double m = (q.y - p.y) / (q.x - p.x);
  const double b = p.y - (m * p.x);

  double x = (m * toProject.y + toProject.x - m * b) / (m * m + 1);
  double y = (m * m * toProject.y + m * toProject.x + b) / (m * m + 1);

  return Point2D(x, y);
}

bool Line::intersectionPoint(const Line& another, Point2D& isecPoint) const {
  const auto& a = p;
  const auto& b = q;
  const auto& c = another.p;
  const auto& d = another.q;

  double s, t;  /* The two parameters of the parametric eqns. */
  double denom; /* Denoninator of solutions. */

  denom = a.x * (d.y - c.y) + b.x * (c.y - d.y) + d.x * (b.y - a.y) +
          c.x * (a.y - b.y);

  /* If denom is zero, then the line segments are parallel. */
  /* In this case, return false even though the segments might overlap. */
  if (denom == 0.0)
    return false;

  s = (a.x * (d.y - c.y) + c.x * (a.y - d.y) + d.x * (c.y - a.y)) / denom;
  t = -(a.x * (c.y - b.y) + b.x * (a.y - c.y) + c.x * (b.y - a.y)) / denom;

  isecPoint.x = a.x + s * (b.x - a.x);
  isecPoint.y = a.y + s * (b.y - a.y);

  if ((0.0 <= s) && (s <= 1.0) && (0.0 <= t) && (t <= 1.0))
    return true;
  else
    return false;
}
} // namespace Kinemator