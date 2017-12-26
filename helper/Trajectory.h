#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "lib/Geometry.h"
#include <boost/thread/mutex.hpp>

class Physics;

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

    Trajectory(Physics* physics){physics_ = physics;}

    void updateTrajectory();

    Vector2d getPredictedBallPosition(int millis) const;

    Vector2d getPredictedBallVelocity(int millis) const;

    PointInfo getPredictedPointInfo(int millis) const;

    std::vector<PointInfo> getBallTrajectory() const;

    void printTrajectory();

private:

    std::vector<PointInfo> trajectory_;
    mutable boost::mutex trajectoryMutex_;

    Physics* physics_;

    const double rollDeceleration = 0.08;
    const double reflectionFactor = 0.38;
};

#endif // TRAJECTORY_H
