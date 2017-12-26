#include "Line.h"

#include <iostream>

Line::Line()
    :
      supportVector_(0,0),
      directionVector_(0,0),
      normalVector_(0,0)
{
}

Line::Line(const Line& line)
    :
      supportVector_(line.getSupportVector()),
      directionVector_(line.getDirectionVector()),
      normalVector_(line.getNormalVector())
{
}

Line::Line(Position pos1, Position pos2)
    :
      supportVector_(pos1),
      directionVector_((Vector2d(pos2) - supportVector_).getNormalized())
{
    normalVector_.x = - directionVector_.y;
    normalVector_.y = directionVector_.x;
}

Line::Line(Position pos, double angle)
    :
      supportVector_(pos),
      directionVector_(angle)
{
    normalVector_.x = - directionVector_.y;
    normalVector_.y = directionVector_.x;
}

Line::Line(Vector2d vec1, Vector2d vec2)
    :
      supportVector_(vec1),
      directionVector_((vec2 - supportVector_).getNormalized())
{
    normalVector_.x = - directionVector_.y;
    normalVector_.y = directionVector_.x;
}

Line::Line(Vector2d vec, double angle)
    :
      supportVector_(vec),
      directionVector_(angle)
{
    normalVector_.x = - directionVector_.y;
    normalVector_.y = directionVector_.x;
}

Line::Line(Position pos, Vector2d direction)
    :
      supportVector_(pos),
      directionVector_(direction.getNormalized())
{
    normalVector_.x = - directionVector_.y;
    normalVector_.y = directionVector_.x;
}

double Line::getDistance(const Position& pos) const
{
    return getDistance(Vector2d(pos));
}

double Line::getDistance(const Vector2d& vec) const
{
    return fabs((vec - supportVector_) * normalVector_);
}

bool Line::isLeftOfLine(const Vector2d &vec) const
{
    return ((vec - supportVector_) * normalVector_ < 0); //For game Field
}

bool Line::isLeftOfLine(const Position &pos) const
{
    return isLeftOfLine(Vector2d(pos));
}

bool Line::intersects(const Line& line) const
{
    return fabs(directionVector_ * line.getDirectionVector()) < 0.99999999;
}

boost::optional<Vector2d> Line::getIntersection(const Line& line) const
{
    if (!intersects(line))
    {
        return boost::none;
    }
    double dist = (line.getSupportVector() - supportVector_) * normalVector_;
    double l = - (line.getDirectionVector() * normalVector_);
    Vector2d vec(line.getDirectionVector() * (dist / l) + line.getSupportVector());
    return vec;
}

Vector2d Line::getClosestPoint(const Position& pos) const
{
    return getClosestPoint(Vector2d(pos));
}

Vector2d Line::getClosestPoint(const Vector2d& vec) const
{
    return supportVector_ + directionVector_ * ((vec - supportVector_) * directionVector_);
}

Vector2d Line::getReflection(const Line& line) const
{
//    double angle = fabs(getAngle(line));

//    if (angle > M_PI/2)
//        angle = M_PI - angle;

//    Vector2d temp(directionVector_);
//    temp.turn(angle);

//    if (temp * normalVector_ < 0)
//    {
//        return temp;
//    }
//    return temp*-1;

    return directionVector_.getTurned(getAngle(line));

}

//Foreign overloading

std::ostream& operator<<(std::ostream& os, const Line& line)
{
    os << "SupportVector:   " << line.getSupportVector() << std::endl;
    os << "DirectionVector: " << line.getDirectionVector() << std::endl;
    return os;
}
