#include "Quadrangle.h"

Quadrangle::Quadrangle(const Quadrangle& quad)
    :
      Obstacle(quad)
{
    segments_ = quad.getSegments();
}

Quadrangle::Quadrangle(Position pos1, Position pos2, Position pos3, Position pos4)
{
    segments_.push_back(LineSegment(pos1, pos2));
    segments_.push_back(LineSegment(pos2, pos3));
    segments_.push_back(LineSegment(pos3, pos4));
    segments_.push_back(LineSegment(pos4, pos1));
    center_ = (((Vector2d(pos1) + Vector2d(pos2) + Vector2d(pos3) + Vector2d(pos4))/4).toPosition());
}

double Quadrangle::getDistance(const Position& pos) const
{
    if (segments_.size() != 4)
    {
        std::cout << "Quadrangle " << getName() << " not defined correctly!" << std::endl;
        return 0;
    }
    return std::min<double>(std::min<double>(segments_.at(0).getDistanceExtended(pos),
                                             segments_.at(1).getDistanceExtended(pos)),
                            std::min<double>(segments_.at(2).getDistanceExtended(pos),
                                             segments_.at(3).getDistanceExtended(pos)));
}

bool Quadrangle::isInside(const Position& pos) const
{
    if (segments_.size() != 4)
    {
        std::cout << "Quadrangle " << getName() << " not defined correctly!" << std::endl;
        return true;
    }
    return segments_.at(0).isLeftOfLine(pos) && segments_.at(1).isLeftOfLine(pos) &&
           segments_.at(2).isLeftOfLine(pos) && segments_.at(3).isLeftOfLine(pos);
}

std::vector<Vector2d> Quadrangle::getIntersection(const Line& line) const
{
    std::vector<Vector2d> intersections;
    for (auto &k : segments_)
    {
        if (boost::optional<Vector2d> intersection = k.getIntersection(line))
        {
            intersections.push_back(*intersection);
        }
    }
    return intersections;
}

std::vector<Vector2d> Quadrangle::getIntersection(const LineSegment& seg) const
{
    std::vector<Vector2d> intersections;
    for (auto &k : segments_)
    {
        if (boost::optional<Vector2d> intersection = k.getIntersection(seg))
        {
            intersections.push_back(*intersection);
        }
    }
    return intersections;
}

bool Quadrangle::intersects(const Line& line) const
{
    for (auto &k : segments_)
    {
        if (k.intersects(line))
        {
            return true;
        }
    }
    return false;
}

bool Quadrangle::intersects(const LineSegment& seg) const
{
    for (auto &k : segments_)
    {
        if (k.intersects(seg))
        {
            return true;
        }
    }
    return false;
}


std::vector<Vector2d> Quadrangle::getTangentPoints(const Position& pos) const
{
    Line line(pos, center_);

    double angleLeft = 0;
    double angleRight = 0;
    Vector2d left = center_;
    Vector2d right = center_;

    std::vector<Vector2d> result;

    for (auto &k : segments_)
    {
//        bool leftOfLine = line.isLeftOfLine(k.getStartPoint());
//        if (leftOfLine)
//        {
//            double angle = line.getAngle(Line(k.getStartPoint(),pos));
//            if (fabs(angle) > angleLeft)
//            {
//                angleLeft = angle;
//                left = k.getStartVector();
//            }
//        }
//        else
//        {
//            double angle = line.getAngle(Line(k.getStartPoint(),pos));
//            if (fabs(angle) > angleRight)
//            {
//                angleRight = angle;
//                right = k.getStartVector();
//            }
//        }

        result.push_back(k.getStartPoint());
    }


//    result.push_back(left);
//    result.push_back(right);
    return result;
}

Position Quadrangle::getValidPosition(const Position& pos) const
{
  Vector2d validPoint(pos);
  double closestDistance = 1000;

  if (isInside(pos))
  {
    for (auto &k : segments_)
    {
      if(k.getDistance(pos)<closestDistance)
      {
        closestDistance = k.getDistance(pos);
        validPoint = Vector2d(pos) + (k.getNormalVector()*(closestDistance+0.05));
      }
    }
  }
  return validPoint.toPosition();
}

std::ostream& operator<<(std::ostream& os, const Quadrangle& quad)
{
    os << "Obstacle: " << quad.getName() << " at: " << Vector2d(quad.getCenter()) << std::endl;
    int i = 0;
    for (auto &k : quad.getSegments())
    {
        os << "Segment " << ++i << ":";
        os << k;
    }
    return os;
}
