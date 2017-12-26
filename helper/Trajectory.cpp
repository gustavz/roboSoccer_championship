#include "Trajectory.h"
#include <math.h>
#include "lib/Physics.h"

#include <iomanip>

void Trajectory::updateTrajectory()
{
    boost::lock_guard<boost::mutex> guard(trajectoryMutex_);
    trajectory_.clear();

    Quadrangle* gameField(physics_->getFieldPtr());
    PointInfo startPoint(physics_->getBallPositionFiltered(), physics_->getBallVelocity());
    trajectory_.push_back(startPoint);

    bool stop = false;

    while (!stop)
    {
        Vector2d s0 = trajectory_.back().Point;
        Vector2d v0 = trajectory_.back().Velocity;
        int t0 = trajectory_.back().Time;

        Line ballTraj(s0, v0.getAngle());
        std::vector<Vector2d> intersections = gameField->getIntersection(ballTraj);

        PointInfo nextPoint;
        if (intersections.size()==0)
        {
            break;
        }
        else if (intersections.size()==1)
        {
            nextPoint.Point = intersections[0];
        }
        else
        {
            if (((intersections[0]-s0)*v0) > 0)
            {
                nextPoint.Point = intersections[0];
            }
            else
            {
                nextPoint.Point = intersections[1];
            }
        }



        double distance = (nextPoint.Point-s0).getLength();
        double radicand = pow(v0.getLength()/rollDeceleration, 2) - 2*distance/rollDeceleration;

        if (radicand >= 0.)
        {
            double nextTime = v0.getLength()/rollDeceleration - sqrt(radicand);
            double endVelocity = v0.getLength() - nextTime*rollDeceleration;

            double closestDistance = 100;
            LineSegment* closestSegment = NULL;

            for (auto &k : physics_->getFieldPtr()->getSegments())
            {
                double currentDistance = k.getDistance(nextPoint.Point);
                if (currentDistance < closestDistance)
                {
                    closestDistance = currentDistance;
                    closestSegment = &k;
                }
            }

            Vector2d reflectionDirection = closestSegment->getReflection(ballTraj);

            nextPoint.Time = t0 + round(nextTime * 1000);
            nextPoint.Velocity =  reflectionDirection * endVelocity * reflectionFactor;

            trajectory_.push_back(nextPoint);
        }
        else
        {
            PointInfo stopPos;
            double stopDistance = pow(v0.getLength(),2)/(2*rollDeceleration);
            stopPos.Point = s0 + (v0.getNormalized() * stopDistance);
            stopPos.Velocity = Vector2d();
            stopPos.Time = t0 + v0.getLength()/rollDeceleration*1000;

            trajectory_.push_back(stopPos);
            stop = true;
        }
    }

}

Vector2d Trajectory::getPredictedBallPosition(int millis) const
{
    boost::lock_guard<boost::mutex> guard(trajectoryMutex_);

    uint i = 0;
    while ((i < trajectory_.size()-1) && (trajectory_[i+1].Time < millis))
    {
        i++;
    }
    if (i == trajectory_.size()-1)
    {
        return trajectory_.back().Point;
    }

    Vector2d s0 = trajectory_[i].Point;
    int t0 = trajectory_[i].Time;
    Vector2d v0 = trajectory_[i].Velocity;
    double deltaT = (millis - t0)/1000.;

    Vector2d predBallPos = s0 + ((v0*deltaT) - (v0.getNormalized()*rollDeceleration*0.5*pow(deltaT, 2)));

    return predBallPos;
}

Vector2d Trajectory::getPredictedBallVelocity(int millis) const
{
    boost::lock_guard<boost::mutex> guard(trajectoryMutex_);
    uint i = 0;
    while ((i < trajectory_.size()-1) && (trajectory_[i+1].Time < millis))
    {
        i++;
    }
    if (i == trajectory_.size()-1)
    {
        return trajectory_.back().Velocity;
    }

    Vector2d s0 = trajectory_[i].Point;
    int t0 = trajectory_[i].Time;
    Vector2d v0 = trajectory_[i].Velocity;
    double deltaT = (millis - t0)/1000.;

    Vector2d predBallVel = v0 - (v0.getNormalized()*deltaT*rollDeceleration);

    return predBallVel;
}

PointInfo Trajectory::getPredictedPointInfo(int millis) const
{
    boost::lock_guard<boost::mutex> guard(trajectoryMutex_);
    uint i = 0;
    while ((i < trajectory_.size()-1) && (trajectory_[i+1].Time < millis))
    {
        i++;
    }
    if (i == trajectory_.size()-1)
    {
        return trajectory_.back();
    }

    Vector2d s0 = trajectory_[i].Point;
    int t0 = trajectory_[i].Time;
    Vector2d v0 = trajectory_[i].Velocity;
    double deltaT = (millis - t0)/1000.;

    PointInfo info;
    info.Point = s0 + ((v0*deltaT) - (v0.getNormalized()*rollDeceleration*0.5*pow(deltaT, 2)));
    info.Velocity = v0 - (v0.getNormalized()*deltaT*rollDeceleration);
    info.Time = millis;

    return info;
}

void Trajectory::printTrajectory()
{
    boost::lock_guard<boost::mutex> guard(trajectoryMutex_);
    std::cout << std::fixed << std::setprecision(3);
    for (auto &k : trajectory_)
    {
        std::cout << "Point: " << k.Point << "; Speed: " << k.Velocity << "; Time: " << k.Time << std::endl;
    }
}

std::vector<PointInfo> Trajectory::getBallTrajectory() const
{
    boost::lock_guard<boost::mutex> guard(trajectoryMutex_);
    return trajectory_;
}
