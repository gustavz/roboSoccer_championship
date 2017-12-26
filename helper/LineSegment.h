#ifndef LINESEGMENT_H
#define LINESEGMENT_H

#include "Line.h"
#include "Vector2d.h"

class LineSegment : public Line
{
public:

    /**
     *@brief default constructor of LineSegment
     */
    LineSegment();

    /**
     *@brief copy constructor of LineSegment
     */
    LineSegment(const LineSegment& seg);

    /**
     *@brief constructing a line segment using 2 positions
     *@param pos1: position 1
     *@param pos2: position 2
     */
    LineSegment(Position pos1, Position pos2);

    /**
     *@brief constructing a line segment using 2 vectors
     *@param vec1: vector 1,2
     */
    LineSegment(Vector2d vec1, Vector2d vec2);

    /**
     *@brief constructing a line segment using position, angle and length
     *@param pos: position
     *@param angle: angle
     *@param length: length
     */
    LineSegment(Position pos, double angle, double length);

    /**
     *@brief constructing a line segment using vector, angle and length
     *@param vec: vector
     *@param angle: angle
     *@param length: length
     */
    LineSegment(Vector2d vec, double angle, double length);

    /**
     *@brief calculates distance to line segment
     *@param pos: position
     */
    double getDistanceExtended(Position pos) const;
    double getDistanceExtended(const Vector2d& vec) const;
    bool isInSegmentArea(const Vector2d& vec) const;
    bool intersects(const Line& line) const;
    bool intersects(const LineSegment& line) const;

    /**
     *@brief getter for starting point of the line segment
     */
    Position getStartPoint() const
    {
        return supportVector_.toPosition();
    }

    /**
     *@brief getter for end point of the line segment
     */
    Position getEndPoint() const
    {
        return endVector_.toPosition();
    }

    /**
     *@brief getter for middle point of the line segment
     */
    Position getMiddlePoint() const
    {
        return getMiddleVector().toPosition();
    }

    /**
     *@brief getter for start vector of the line segment
     */
    Vector2d getStartVector() const
    {
        return supportVector_;
    }

    /**
     *@brief getter for end vector of the line segment
     */
    Vector2d getEndVector() const
    {
        return endVector_;
    }

    /**
     *@brief getter for middle vector of the line segment
     */
    Vector2d getMiddleVector() const
    {
        return (supportVector_ + endVector_) / 2;
    }

    /**
     *@brief getter for the length of the line segment
     */
    double getLength() const
    {
        return length_;
    }

    /**
     *@brief getter for intersection of line segment and line
     */
    boost::optional<Vector2d> getIntersection(const Line& line) const;

    /**
     *@brief getter for intersection of two line segments
     */
    boost::optional<Vector2d> getIntersection(const LineSegment& seg) const;

    /**
     *@brief calculates the closest point on the line segment to the given Position
     */
    Vector2d getClosestPoint(const Position& pos) const;

    /**
     *@brief calculates the closest point on the line segment to the given Vector
     */
    Vector2d getClosestPoint(const Vector2d& vec) const;

    friend std::ostream& operator<<(std::ostream& os, const LineSegment& vec);

private:

    Vector2d endVector_; /**< vector of end point of line segment */

    double length_; /**< length of line segment*/
};

#endif // LINESEGMENT_H
