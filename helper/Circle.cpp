#include "Circle.h"

Circle::Circle()
{
}

Circle::Circle(const Circle& circle)
    :
      Obstacle(circle),
      radius_(circle.getRadius())
{
}

Circle::Circle(const Position& center, double r)
    :
      Obstacle(center),
      radius_(r)
{
}

Circle::Circle(const Vector2d& center, double r)
    :
      Obstacle(center.toPosition()),
      radius_(r)
{
}

std::vector<Vector2d> Circle::getIntersection(const Line& line) const
{
    std::vector<Vector2d> intersections;
    double dist = line.getDistance(center_);
    if (dist > radius_)
    {
        return intersections;
    }

    double length = line.getDirectionVector() * (Vector2d(center_) - line.getSupportVector());

    if (dist >= radius_ * 0.99)
    {
        intersections.push_back(line.getSupportVector() + (line.getDirectionVector() * length));
        return intersections;
    }

    double secante = sin(acos(dist / radius_)) * radius_;

    intersections.push_back(line.getSupportVector() +
                line.getDirectionVector() * (length - secante));
    intersections.push_back(line.getSupportVector() +
                line.getDirectionVector() * (length + secante));

    return intersections;
}

std::vector<Vector2d> Circle::getIntersection(const LineSegment& seg) const
{
    std::vector<Vector2d> inter = getIntersection(static_cast<Line>(seg));
    std::vector<Vector2d> intersections;

    for (auto &k: inter)
    {
        if (seg.isInSegmentArea(k))
        {
            intersections.push_back(k);
        }
    }
    return intersections;
}

bool Circle::intersects(const Line& line) const
{
    return line.getDistance(center_) <= radius_;
}

bool Circle::intersects(const LineSegment& seg) const
{
    std::vector<Vector2d> inter = getIntersection(static_cast<Line>(seg));

    for (auto &k: inter)
    {
        if (seg.isInSegmentArea(k))
        {
            return true;
        }
    }
    return false;
}

std::vector<Vector2d> Circle::getTangentPoints(const Position& pos) const
{
    LineSegment connection(center_, pos);
    // add tolerance
    double radiusWithTol = radius_*1.4;
    double angle = acos(radiusWithTol / connection.getLength());
    double conAngle = connection.getDirectionVector().getAngle();

    std::vector<Vector2d> result;
    result.push_back(Vector2d(center_) + (Vector2d(conAngle + angle) * radiusWithTol));
    result.push_back(Vector2d(center_) + (Vector2d(conAngle - angle) * radiusWithTol));

    return result;
}

double Circle::getDistance(const Position& pos) const
{
    return center_.getDistance(pos);
}

bool Circle::isInside(const Position& pos) const
{
    return getDistance(pos) < radius_;
}

Position Circle::getValidPosition(const Position& pos) const
{
  Vector2d validPoint(pos);
  Vector2d center(center_);
  if (isInside(pos))
    validPoint = center + ((validPoint - center).getNormalized()*radius_*1.4);
  return validPoint.toPosition();
}

std::ostream& operator<<(std::ostream& os, const Circle& circle)
{
    os << "Obstacle: " << circle.getName() << " at: " << Vector2d(circle.getCenter()) << std::endl;
    os << "Radius: " << circle.getRadius() << std::endl;
    return os;
}
