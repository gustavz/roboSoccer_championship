#ifndef VECTOR2D_H
#define VECTOR2D_H

#include "position.h"
#include <vector>

class Vector2d
{
public:

    /**
     *@brief Default constructor of Vector2d
     */
    Vector2d();

    /**
     *@brief Copy constructor of Vector2d
     */
    Vector2d(const Vector2d& vec);

    /**
     *@brief Constructing a Vector using coordinates
     *@param newX: X coordinate of Vector
     *@param newY: Y coordinate of Vector
     */
    Vector2d(double newX, double newY);

    /**
     *@brief Constructing a Vector using a position
     *@param pos: Position to be turned to a Vector
     */
    Vector2d(Position const& pos);

    /**
     *@brief Constructing a normalized Vector using an angle
     *@param angle: angle of the new Vector
     */
    Vector2d(double angle);

    /**
     *@brief Calculates length of the Vector
     *@return Length
     */
    double getLength() const;

    /**
     *@brief Calculates length squared of the Vector
     *@return Length Squared
     */
    double getLengthSquared() const;

    /**
     *@brief Calculates distance to another Vector
     *@param vec: Other Vector
     *@return Distance
     */
    double getDistance(const Vector2d& vec) const;

    /**
     *@brief Calculates distance to a Position
     *@param pos: Position
     *@return Distance
     */
    double getDistance(const Position& pos) const;

    /**
     *@brief Calculates angle between this Vector and another one.
     *@param vec: Other Vector
     *@return Angle between Vectors [rad]
     */
    double getAngle(const Vector2d& vec) const;

    /**
     *@brief Calculates angle of this Vector.
     *@return Returns angle of the Vector
     */
    double getAngle() const;

    /**
     *@brief Normalizes the Vector.
     */
    void normalize();

    /**
     *@brief Turns the Vector by an angle [rad].
     *@param rad: Angle in radiant
     */
    void turn(double rad);

    /**
     *@brief Calculates normalized Vector in direction of this Vector.
     *@return The normalized Vector
     */
    Vector2d getNormalized() const;

    /**
     *@brief Calculates turned Vector in mathmatical positive direction.
     *@return The turned Vector
     */
    Vector2d getTurned(double rad) const;

    /**
     *@brief Converts the Vector to a Position
     *@return The conversion
     */
    Position toPosition() const;

    /**
     *@brief Overwrites the + Operator
     *@param rhs: Vector for operation
     *@return Added Vectors
     */
    Vector2d operator+(Vector2d const& rhs) const;

    /**
     *@brief Overwrites the - Operator
     *@param rhs: Vector for operation
     *@return Subtracted Vectors
     */
    Vector2d operator-(Vector2d const& rhs) const;

    /**
     *@brief Overwrites the * Operator with a double value
     *@param times: Double value for operation
     *@return Scaled Vector
     */
    Vector2d operator*(double times) const;

    /**
     *@brief Overwrites the / Operator with a double value
     *@param divide: Double value for operation
     *@return Scaled Vector
     */
    Vector2d operator/(double divide) const;

    /**
     *@brief Overwrites the * Operator with another Vector
     *@param rhs: Other Vector for dot-product
     *@return  Scalar dot-product
     */
    double operator*(Vector2d const& rhs) const;

    friend std::ostream& operator<<(std::ostream& os, const Vector2d& vec);
    friend std::ostream& operator<<(std::ostream& os, const std::vector<Vector2d>& vec);

    // Stuff for kd tree
    union {
        struct {
            double x;
            double y;
        };
        double d[2];
    };

    double& operator [] (unsigned int index);
    double operator [] (unsigned int index) const;
};

#endif // VECTOR2D_H
