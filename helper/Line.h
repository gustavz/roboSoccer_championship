#ifndef LINE_H
#define LINE_H

#include "Vector2d.h"
#include <boost/optional.hpp>

class Line
{
public:

    /**
     *@brief default Constructor of Line
     */
    Line();

    /**
     *@brief copy Constructor of Line
     */
    Line(const Line& line);

    /**
     *@brief constructing a line using two positions
     *@param pos1,2: Position 1,2
     */
    Line(Position pos1, Position pos2);

    /**
     *@brief constructing a line using positon and angle
     *@param pos: Position
     *@param angle: Angle
     */
    Line(Position pos, double angle);

    /**
     *@brief constructing a line using two vectors
     *@param vec1,2: Vector 1,2
     */
    Line(Vector2d vec1, Vector2d vec2);

    /**
     *@brief constructing a line using vector and angle
     *@param vec: Vector
     *@param angle: Angle
     */
    Line(Vector2d vec, double angle);

    /**
     *@brief constructing a line using a position and a direction vector
     *@param pos: position vector
     *@param direction: direction vector
     */
    Line(Position pos, Vector2d direction);

    /**
     *@brief getter for Distance of line to Position
     */
    double getDistance(const Position& pos) const;

    /**
     *@brief getter for Distance of line to vector
     */
    double getDistance(const Vector2d& vec) const;

    /**
     *@brief Checks if position is left of line
     */
    bool isLeftOfLine(const Vector2d& vec) const;

    /**
     *@brief Checks if position is left of line
     */
    bool isLeftOfLine(const Position& pos) const;

    /**
     *@brief Checks intesection: False if line is parallel, True else
     */
    bool intersects(const Line& line) const;

    /**
     *@brief getter for support vector of line
     */
    Vector2d getSupportVector() const
    {
        return supportVector_;
    }

    /**
     *@brief getter for direction vector of line
     */
    Vector2d getDirectionVector() const
    {
        return directionVector_;
    }

    /**
     *@brief getter for normal vector of line (points left)
     */
    Vector2d getNormalVector() const
    {
        return normalVector_;
    }

    /**
     *@brief sets support vector of line using a vector
     *@param vec: Vector
     */
    void setSupportVector(const Vector2d& vec)
    {
        supportVector_ = vec;
    }

    /**
     *@brief sets support vector of line using a position
     *@param pos: Position
     */
    void setSupport(const Position& pos)
    {
        supportVector_ = Vector2d(pos);
    }

    /**
     *@brief sets direction vector of line using a vector
     *@param vec: Vector
     */
    void setDirectionVector(const Vector2d& vec)
    {
        directionVector_ = vec;
        directionVector_.normalize();
        normalVector_.x = - directionVector_.y;
        normalVector_.y = directionVector_.x;
    }

    /**
     *@brief sets direction vector of line using an angle
     *@param phi: angle
     */
    void setDirection(double phi)
    {
        directionVector_.x = cos(phi);
        directionVector_.y = sin(phi);
        normalVector_.x = - directionVector_.y;
        normalVector_.y = directionVector_.x;
    }

    /**
     *@brief getter for angle of line
     */
    double getAngle(const Line& line) const
    {
        return directionVector_.getAngle(line.getDirectionVector());
    }

    /**
     *@brief calculates the Intesection of two lines
     */
    boost::optional<Vector2d> getIntersection(const Line& line) const;

    /**
     *@brief calculates the closest Point on the line to the given Position
     */
    Vector2d getClosestPoint(const Position& pos) const;

    /**
     *@brief calculates the closest Point on the line to the given Vector
     */
    Vector2d getClosestPoint(const Vector2d& vec) const;


    /**
     *@brief getter for reflection vector of line reflecting at a line segment
     */
    Vector2d getReflection(const Line& line) const;

    friend std::ostream& operator<<(std::ostream& os, const Line& vec);

protected:

    Vector2d supportVector_; /**< support vector of the line */
    Vector2d directionVector_; /**< direction of the line, always normalized */
    Vector2d normalVector_; /**< normal vector (orthogonal to directionVector), right-oriented */

};

#endif // LINE_H
