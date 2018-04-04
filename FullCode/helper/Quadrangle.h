#ifndef QUADRANGLE_H
#define QUADRANGLE_H

#include "Obstacle.h"
#include "LineSegment.h"
#include "Vector2d.h"

#include <vector>

struct Quadrangle : public Obstacle
{
public:

    /**
     *@brief Copy constructor of Quadrangle
     */
    Quadrangle(const Quadrangle& quad);

    /**
     *@brief Constructing a Quadrangle using 4 Positions. Defined counter clockwise!
     *@param pos1,...,pos4: Position 1 - 4
     */
    Quadrangle(Position pos1, Position pos2, Position pos3, Position pos4);

    virtual double getDistance(const Position& pos) const;
    virtual bool isInside(const Position& pos) const;

    /**
     *@brief getter for line segments of a quadrangle
     */
    std::vector<LineSegment> getSegments() const{ return segments_;}

    virtual std::vector<Vector2d> getIntersection(const Line& line) const;
    virtual std::vector<Vector2d> getIntersection(const LineSegment& seg) const;
    virtual bool intersects(const Line& line) const;
    virtual bool intersects(const LineSegment& seg) const;
    virtual std::vector<Vector2d> getTangentPoints(const Position& pos) const;
    void setCenter(const Position&) {}

    virtual Position getValidPosition(const Position& pos) const;

    friend std::ostream& operator<<(std::ostream& os, const Quadrangle& vec);

private:

	std::vector<LineSegment> segments_; /**< line segments forming quadrangle */
};

#endif // QUADRANGLE_H
