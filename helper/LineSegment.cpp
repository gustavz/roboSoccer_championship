#include "LineSegment.h"
#include <math.h>
#include <iostream>

LineSegment::LineSegment()
    :
      Line(),
      endVector_(0,0),
      length_(0)
{
}

LineSegment::LineSegment(const LineSegment& seg)
    :
      Line(seg),
      endVector_(seg.getEndVector()),
      length_(seg.getLength())
{
}

LineSegment::LineSegment(Position pos1, Position pos2)
    :
      Line(pos1, pos2),
      endVector_(pos2),
      length_(endVector_.getDistance(supportVector_))
{
}

LineSegment::LineSegment(Vector2d vec1, Vector2d vec2)
    :
      Line(vec1, vec2),
      endVector_(vec2),
      length_(endVector_.getDistance(supportVector_))
{
}

LineSegment::LineSegment(Position pos, double angle, double length)
    :
      Line(pos, angle),
      length_(length)
{
    endVector_ = supportVector_ + directionVector_ * length_;
}

LineSegment::LineSegment(Vector2d vec, double angle, double length)
    :
      Line(vec, angle),
      length_(length)
{
    endVector_ = supportVector_ + directionVector_ * length_;
}


double LineSegment::getDistanceExtended(Position pos) const
{
    return getDistanceExtended(Vector2d(pos));
}

double LineSegment::getDistanceExtended(const Vector2d& vec) const
{
    double distToSupportVector = (vec - supportVector_) * directionVector_;
    if (distToSupportVector < 0)
    {
        return supportVector_.getDistance(vec);
    }
    if (distToSupportVector > length_)
    {
        return endVector_.getDistance(vec);
    }
    return getDistance(vec);
}

bool LineSegment::isInSegmentArea(const Vector2d& vec) const
{
    double dist = directionVector_ * (vec - supportVector_);
    return (dist >= 0 && dist <= length_);
}

bool LineSegment::intersects(const Line& line) const
{
    return (line.isLeftOfLine(getStartPoint()) != line.isLeftOfLine(getEndPoint()));
}

bool LineSegment::intersects(const LineSegment& seg) const
{
    return seg.intersects(static_cast<const Line>(*this)) && intersects(static_cast<const Line>(seg));
}

boost::optional<Vector2d> LineSegment::getIntersection(const Line& line) const
{
    if (!intersects(line))
    {
        return boost::none;
    }
    return line.getIntersection(*this);
}

boost::optional<Vector2d> LineSegment::getIntersection(const LineSegment& seg) const
{
    if (!intersects(static_cast<Line>(seg)))
    {
        return boost::none;
    }
    return seg.getIntersection(static_cast<const Line>(*this));
}

Vector2d LineSegment::getClosestPoint(const Position& pos) const
{
    return getClosestPoint(Vector2d(pos));
}

Vector2d LineSegment::getClosestPoint(const Vector2d& vec) const
{
    double dotProduct = (vec - supportVector_) * directionVector_;
    if (dotProduct <= 0)
    {
        return supportVector_;
    }

    if (dotProduct >= length_)
    {
        return endVector_;
    }

    return supportVector_ + directionVector_ * dotProduct;
}

std::ostream& operator<<(std::ostream& os, const LineSegment& seg)
{
    os << "StartVector:     " << seg.getSupportVector() << std::endl;
    os << "EndVector:       " << seg.getEndVector() << std::endl;
    os << "Length:          " << seg.getLength() << std::endl;
    os << "DirectionVector: " << seg.getDirectionVector() << std::endl;
    return os;
}
