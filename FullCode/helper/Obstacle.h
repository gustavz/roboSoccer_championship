#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "Line.h"
#include "LineSegment.h"
#include "Vector2d.h"

#include <iostream>

class Obstacle
{
public:

    /**
     *@brief default constructor of Obstacle
     */
    Obstacle(){}

    /**
     *@brief copy constructor of Obstacle
     */
    Obstacle(const Obstacle& obst): center_(obst.getCenter()), name_(obst.getName()){}

    /**
     *@brief constructing an obstacle using its position
     *@param pos: position as center of obstacle
     */
    Obstacle(const Position& pos) : center_(pos){}

    /**
     *@brief constructing an obstacle using its name
     *@param name: name of obstacle
     */
    Obstacle(string name) : name_(name) {}

    /**
     *@brief constructing an obstacle using its position and its name
     *@param pos: position as center of obstacle
     *@param name: name of obstacle
     */
    Obstacle(const Position& pos, string name) : center_(pos), name_(name) {}

    /**
     *@brief getter for name of obstacle
     */
    string getName() const {return name_;}

    /**
     *@brief sets name of obstacle
     */
    void setName(string name) {name_ = name;}

    /**
     *@brief getter for distance of obstacle to position
     *@param p: position
     */
    virtual double getDistance(const Position& p) const = 0;

    /**
     *@brief determines if position is inside an obstacle
     *@param p: position
     */
    virtual bool isInside(const Position& p) const = 0;

    /**
     *@brief calculates intersection points between a line and an obstacle
     *@param line: line
     */
    virtual std::vector<Vector2d> getIntersection(const Line& line) const = 0;

    /**
     *@brief calculates intersection points between a line segment and an obstacle
     *@param seg: line segment
     */
    virtual std::vector<Vector2d> getIntersection(const LineSegment& seg) const = 0;

    /**
     *@brief determines if there is an intersection between a line and an obstacle
     *@param line: line
     */
    virtual bool intersects(const Line& line) const = 0;

    /**
     *@brief determines if there is an intersection between a line segment and an obstacle
     *@param seg: line segment
     */
    virtual bool intersects(const LineSegment& seg) const = 0;

    /**
     *@brief sets center of an obstacle
     *@param pos: new center position
     */
    void setCenter(const Position& pos) {center_ = pos;}

    /**
     *@brief sets center of an obstacle
     *@param vec: new center position
     */
    void setCenter(const Vector2d& vec) {center_ = vec.toPosition();}

    /**
     *@brief getter for center of an obstacle
     */
    Position getCenter() const {return center_.toPosition();}

    /**
     *@brief calculates tangent points of an obstacle
     *@param pos: point the tangents shall run through
     */
    virtual std::vector<Vector2d> getTangentPoints(const Position& pos) const = 0;

    /**
     *@brief gets the closest valid position (position outside an obstacle)
     *@param pos: old position that shall be checked and, if necessary, updated
     */
    virtual Position getValidPosition(const Position& pos) const = 0;

    friend std::ostream& operator<<(std::ostream& os, const Obstacle& vec);

protected:
    Vector2d center_; /**< center of obstacle */
    string name_; /**< name of obstacle */
};

#endif // OBSTACLE_H
