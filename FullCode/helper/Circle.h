#ifndef CIRCLE_H
#define CIRCLE_H

#include "Obstacle.h"
#include "Vector2d.h"

class Circle : public Obstacle
{
public:

    /**
     *@brief default Constructor of Circle
     */
    Circle();

    /**
     *@brief copy Constructor of Circle
     */
    Circle(const Circle& circle);

    /**
     *@brief Constructs a circle
     *@param center: New center of circle
     *@param r: New radius of circle
     */
    Circle(const Position& center, double r);

    /**
     *@brief Constructs a circle
     *@param center: New center of circle
     *@param r: New radius of circle
     */
    Circle(const Vector2d& center, double r);

    /**
     *@brief Getter for radius of Circle
     */
    double getRadius() const {return radius_;}
    virtual double getDistance(const Position& pos) const;
    virtual bool isInside(const Position& pos) const;

    virtual std::vector<Vector2d> getIntersection(const Line& line) const;
    virtual std::vector<Vector2d> getIntersection(const LineSegment& seg) const;
    virtual bool intersects(const Line& line) const;
    virtual bool intersects(const LineSegment& seg) const;
    virtual std::vector<Vector2d> getTangentPoints(const Position& pos) const;

    virtual Position getValidPosition(const Position& pos) const;

    friend std::ostream& operator<<(std::ostream& os, const Circle& vec);

private:

    double radius_; /**< Radius of the Circle */
};

#endif // CIRCLE_H
