#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "lib/Geometry.h"
#include <boost/thread/mutex.hpp>

class Physics;

/**
  *@brief A struct storing information about a point on the ball trajectory.
  *       Stores Position, Time and Velocity
  */
struct PointInfo
{
    PointInfo(){}

    PointInfo(Vector2d point, Vector2d velocity)
        :
          Point(point),Time(0),Velocity(velocity)
    {}

    PointInfo(Vector2d point, Vector2d velocity, int time)
        :
          Point(point),Time(time),Velocity(velocity)
    {}

    Vector2d Point;
    int Time;
    Vector2d Velocity;

};

class Trajectory
{

public:

    /**
     *@brief Constructor for Trajectory with a pointer to a physics instance
     *@param physics Pointer to physics instance
     */
    Trajectory(Physics* physics){physics_ = physics;}

    /**
     *@brief updates the whole ball trajectory
     */
    void updateTrajectory();

    /**
     *@brief retrieves the predicted ball position after millis
     *@param millis Milliseconds in the future
     */
    Vector2d getPredictedBallPosition(int millis) const;

    /**
     *@brief retrieves the predicted ball velocity after millis
     *@param millis Milliseconds in the future
     */
    Vector2d getPredictedBallVelocity(int millis) const;

    /**
     *@brief retrieves the predicted ball PointInfo after millis
     *@param millis Milliseconds in the future
     */
    PointInfo getPredictedPointInfo(int millis) const;

    /**
     *@brief retrieves the whole ball Trajectory as calculated in updateTrajectory()
     */
    std::vector<PointInfo> getBallTrajectory() const;

    /**
     *@brief print the whole ball trajectory
     */
    void printTrajectory();

private:

    std::vector<PointInfo> trajectory_; /**< vector of collision points in the future*/
    mutable boost::mutex trajectoryMutex_; /**< mutex to lock the trajectory*/

    Physics* physics_; /**< physics instance*/

    const double rollDeceleration = 0.08; /**< Roll Deceleration Factor for trajectory calculation*/
    const double reflectionFactor = 0.38; /**< Reflection factor for speed loss at collision with the boundary*/
};

#endif // TRAJECTORY_H
