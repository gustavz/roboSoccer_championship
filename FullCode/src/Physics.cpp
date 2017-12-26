#include "lib/Physics.h"
#include "lib/Agent.h"
#include "lib/Enemy.h"
#include "raw_ball.h"

#include <iomanip>

#define PHYSICS_UPDATE_INTERVAL 30003

Physics::Physics()
    :
      RunnableObject(0, PHYSICS_UPDATE_INTERVAL),
      gameField_
      (
          //Initialize Field starting left bottom going counter-clockwise
          Quadrangle(Position(-1.385, 0.896), Position(1.425, 0.882),
                     Position(1.468, -0.918), Position(-1.462, -0.880)),
          //GoalLeft
          LineSegment(Position(-1.405, -0.228), Position(-1.391, 0.281)),
          //GoalRight
          LineSegment(Position(1.420, 0.260), Position(1.422, -0.272)),
          //InnerGoalLeft
          LineSegment(Position(-1.462, -0.112), Position(-1.462, 0.165)),
          //InnerGoalRight
          LineSegment(Position(1.483, 0.137), Position(1.488, -0.149)),
          //HalfwayLine
          LineSegment(Position(0.016, 0.869), Position(-0.001, -0.880)),
          // PenaltyAreaLeft
          Quadrangle(Position(-1.386, 0.364), Position(-1.173, 0.363),
                     Position(-1.196, -0.33), Position(-1.412, -0.319)),
          // PenaltyAreaRight
          Quadrangle(Position(1.196, 0.346), Position(1.415, 0.343),
                     Position(1.43, -0.357), Position(1.204, -0.35)),
          // ObstaclePenaltyAreaLeft
          Quadrangle(Position(-10.386, 0.364 + PENALTY_AREA_MARGIN),
                     Position(-1.173 + PENALTY_AREA_MARGIN, 0.363 + PENALTY_AREA_MARGIN),
                     Position(-1.196 + PENALTY_AREA_MARGIN, -0.33 - PENALTY_AREA_MARGIN),
                     Position(-10.412, -0.319 - PENALTY_AREA_MARGIN)),
          // ObstaclePenaltyAreaRight
          Quadrangle(Position(1.196 - PENALTY_AREA_MARGIN, 0.346 + PENALTY_AREA_MARGIN),
                     Position(10.415, 0.343 + PENALTY_AREA_MARGIN),
                     Position(10.43, -0.357 - PENALTY_AREA_MARGIN),
                     Position(1.204 - PENALTY_AREA_MARGIN, -0.35 - PENALTY_AREA_MARGIN)),
          // ObstacleField
          Quadrangle(Position(-1.385 + FIELD_MARGIN, 0.896 - FIELD_MARGIN),
                     Position(1.425 - FIELD_MARGIN, 0.882 - FIELD_MARGIN),
                     Position(1.468 - FIELD_MARGIN, -0.918 + FIELD_MARGIN),
                     Position(-1.462 + FIELD_MARGIN, -0.880 + FIELD_MARGIN)),
          // CornerBottomLeft
          Quadrangle(Position(-1.388, 0.895), Position(-1.118, 0.874),
                     Position(-1.289, 0.804), Position(-1.371, 0.614)),
          // CornerBottomRight
          Quadrangle(Position(1.151, 0.863), Position(1.474, 0.926),
                     Position(1.406, 0.607), Position(1.33, 0.786)),
          // CornerTopRight
          Quadrangle(Position(1.361, -0.816), Position(1.43, -0.655),
                     Position(1.466, -0.917), Position(1.178, -0.894)),
          // CornerTopLeft
          Quadrangle(Position(-1.422, -0.6), Position(-1.355, -0.785),
                     Position(-1.31, -0.86), Position(-1.463, -0.878)),
          // ObstacleCornerBottomLeft
          Quadrangle(Position(-1.388, 0.895), Position(-1.118, 0.874),
                     Position(-1.289 + CORNER_MARGIN, 0.804 - CORNER_MARGIN), Position(-1.371, 0.614)),
          // ObstacleCornerBottomRight
          Quadrangle(Position(1.151, 0.863), Position(1.474, 0.926),
                     Position(1.406, 0.607), Position(1.33 - CORNER_MARGIN, 0.786 - CORNER_MARGIN)),
          // ObstacleCornerTopRight
          Quadrangle(Position(1.361 - CORNER_MARGIN, -0.816 + CORNER_MARGIN), Position(1.43, -0.655),
                     Position(1.466, -0.917), Position(1.178, -0.894)),
          // ObstacleCornerTopLeft
          Quadrangle(Position(-1.422, -0.6), Position(-1.355 + CORNER_MARGIN, -0.785 + CORNER_MARGIN),
                     Position(-1.31, -0.86), Position(-1.463, -0.878)),
          // LeftHalf
          Quadrangle(Position(-1.385, 0.896), Position(0.016, 0.869),
                     Position(-0.001, -0.880), Position(-1.462, -0.880)),
          // RightHalf
          Quadrangle(Position(0.016, 0.869), Position(1.425, 0.882),
                     Position(1.468, -0.918), Position(-0.001, -0.880)),
          // UpperHalf
          Quadrangle(Position(-1.424, 0.021), Position(1.449, 0.003),
                     Position(1.468, -0.918), Position(-1.462, -0.880)),
          // LowerHalf
          Quadrangle(Position(-1.385, 0.896), Position(1.425, 0.882),
                     Position(1.449, 0.003), Position(-1.424, 0.021))
      ),
      ballTrajectory_(this)
{
    gameField_.Field.setName("Field");
    gameField_.PenaltyAreaLeft.setName("PenaltyAreaLeft");
    gameField_.PenaltyAreaRight.setName("PenaltyAreaRight");
    gameField_.ObstaclePenaltyAreaLeft.setName("ObstaclePenaltyAreaLeft");
    gameField_.ObstaclePenaltyAreaRight.setName("ObstaclePenaltyAreaRight");
    gameField_.ObstacleField.setName("ObstacleField");
    gameField_.CornerBottomLeft.setName("CornerBottomLeft");
    gameField_.CornerBottomRight.setName("CornerBottomRight");
    gameField_.CornerTopRight.setName("CornerTopRight");
    gameField_.CornerTopLeft.setName("CornerTopLeft");
    gameField_.ObstacleCornerBottomLeft.setName("ObstacleCornerBottomLeft");
    gameField_.ObstacleCornerBottomRight.setName("ObstacleCornerBottomRight");
    gameField_.ObstacleCornerTopRight.setName("ObstacleCornerTopRight");
    gameField_.ObstacleCornerTopLeft.setName("ObstacleCornerTopLeft");
    gameField_.LeftHalf.setName("LeftHalf");
    gameField_.RightHalf.setName("RightHalf");
    gameField_.UpperHalf.setName("UpperHalf");
    gameField_.LowerHalf.setName("LowerHalf");
    velTimer_.start();
}

Physics::Physics(Ball* ball)
    :
      Physics()
{
    ball_ = ball;
}

void Physics::initializePhysics()
{
    std::cout << "Initialize Physics" << std::endl;

    for (auto i: agents_)
    {
        i->initializePath();
        Circle* agentCircle = new Circle(i->getPositionFiltered(), ROBOT_OBSTACLE_RADIUS);
        string name = "Agent ";
        name += to_string(i->GetRfcommNr());
        agentCircle->setName(name);
        agentObstacles_.push_back(agentCircle);
    }

    for (auto i: enemies_)
    {
        Circle* enemyCircle = new Circle(i->GetPos(), ROBOT_OBSTACLE_RADIUS);
        string name = "Enemy ";
        name += to_string(i->GetRfcommNr());
        enemyCircle->setName(name);
        enemyObstacles_.push_back(enemyCircle);
    }

    ballObstacle_ = new Circle(ball_->GetPos(), BALL_OBSTACLE_RADIUS);
    ballObstacle_->setName("Ball");

    std::cout << "Initialize Physics DONE" << std::endl;
}

void Physics::addAgent(Agent* ag)
{
    agents_.push_back(ag);
}

void Physics::addEnemy(Enemy* em)
{
    enemies_.push_back(em);
}

void Physics::run()
{
    while (status_ == RunStatus::RUN)
    {
        restartTimer();
        updatePhysics();
        updateObstacles();
        PredictBallTrajectory(1000);
        usleep(getSleepTime());
    }
}

void Physics::updatePhysics()
{
    static int wrongValueCounter = 0;
    static int counter = 0;
    static int64_t timeSinceLastPredComparison = 0;

    static Timestamp lastBallTimestamp;




	Vector2d ballPos(ball_->GetPos());
	if(!isInsideGameField(ballPos.toPosition())){
		cout << "Ball is not inside GameField!!!" << endl;
		ballPos = ballPositionFiltered_;
	}

    Timestamp actualTimestamp = ball_->getLastTimestamp();

    int64_t timeDiff = lastBallTimestamp.diffns(actualTimestamp);

    if (timeDiff == 0)
    {
        return;
    }

    movingAverage(ballPositionFiltered_, ballPos, 0.6);

    Vector2d newBallVelocity =
            (ballPositionFiltered_ - lastBallPosition_) / (double(timeDiff) / 1e9);

    movingAverage(ballVelocity_, newBallVelocity, 0.8);

    lastBallPosition_ = ballPositionFiltered_;

    if (comparePrediction_)
    {
        static Timestamp compareTimer = lastBallTimestamp;
        timeSinceLastPredComparison += timeDiff;
        if (timeSinceLastPredComparison >= 100 * 1e6)
        {
            std::cout << std::fixed << std::setprecision(3) << "=============== New Prediction ===============" << std::endl;
            std::cout << "Timestamp: " << compareTimer.diffns(actualTimestamp) / 1e6 << std::endl;
            std::cout << "Predicted Position: " << predictions_.front().Point << std::endl;
            std::cout << "Predicted Velocity: " << predictions_.front().Velocity << std::endl;
            std::cout << "Measured Position: " << ballPositionFiltered_ << std::endl;
            std::cout << "Measured Velocity: " << ballVelocity_ << std::endl;

            std::cout << "Position Error: "  << ballPositionFiltered_.getDistance(predictions_.front().Point) << std::endl;
            std::cout << "Measured Velocity: " << ballVelocity_.getDistance(predictions_.front().Velocity) << std::endl;
            predictions_.erase(predictions_.begin());
            timeSinceLastPredComparison = timeSinceLastPredComparison - 100*1e6;
            if (predictions_.empty())
            {
                compareTimer = actualTimestamp;
                stopComparePrediction();
            }
        }
    }

    lastBallTimestamp = actualTimestamp;



    if (counter > 10)
    {
        ballTrajectory_.updateTrajectory();
    }
    else
    {
        counter++;
    }


}

void Physics::PredictBallTrajectory(int milliseconds)
{
    //Calculate simpleBallTrajectory_
    simpleBallTrajectory_.setSupportVector(lastBallPosition_);
    simpleBallTrajectory_.setDirectionVector(ballVelocity_.getNormalized());
}

Position Physics::getPredBallPosition(int milliseconds) const
{
//    Position temp = (simpleBallTrajectory_.getSupportVector() + (simpleBallTrajectory_.getDirectionVector() * ballVelocity_.getLength() * milliseconds / 1000)).toPosition();
//    if (ballVelocity_.getLength() < 0.05)
//    {
//        temp = lastBallPosition_.toPosition();
//    }
//    return temp;
    Vector2d predBallPos = ballTrajectory_.getPredictedBallPosition(milliseconds);
    return predBallPos.toPosition();
}

Vector2d Physics::getPredBallVelocity(int milliseconds) const
{
    return ballTrajectory_.getPredictedBallVelocity(milliseconds);
}

Vector2d Physics::getBallVelocity() const
{
    return ballVelocity_;
}

Line Physics::getSimpleBallTrajectory() const
{
    return simpleBallTrajectory_;
}

std::vector<LineSegment> Physics::getBallTrajectory(int milliseconds) const
{
    std::vector<LineSegment> ballTraj;
    std::vector<PointInfo> traj = ballTrajectory_.getBallTrajectory();

    Vector2d Point = traj.front().Point;

    for (uint i = 1; i < traj.size(); i++)
    {
        Vector2d nextPoint = traj[i].Point;
        if (traj[i].Time <= milliseconds)
        {
            ballTraj.push_back(LineSegment(Point, nextPoint));
        }
        else
        {
            ballTraj.push_back(LineSegment(Point, getPredBallPosition(milliseconds)));
            break;
        }
        Point = nextPoint;
    }

    return ballTraj;
}

Enemy* Physics::getEnemyClosestToBall()
{
    Enemy* closestEnemy = NULL;
    double closestDist = 1000.;

    for (auto k : enemies_)
    {
        double dist = k->getDistToBall();
        if (dist < closestDist)
        {
            closestEnemy = k;
            closestDist = dist;
        }
    }
    return closestEnemy;
}

double Physics::getClosestEnemysDistance(const Vector2d& vec) const
{
    double closestDist = 1000.;

    for (auto k : enemies_)
    {
        double dist = vec.getDistance(k->GetPos());
        if (dist < closestDist)
        {
            closestDist = dist;
        }
    }
    return closestDist;
}

double Physics::getClosestEnemysDistanceToBall() const
{
    return getClosestEnemysDistance(ballPositionFiltered_);
}


void Physics::startComparePrediction(int predictionTime)
{
    int timePred = 100;
    predictions_.clear();

    std::cout << "New Trajectory" << std::endl;
    ballTrajectory_.printTrajectory();

    while(timePred <= predictionTime)
    {
        predictions_.push_back(ballTrajectory_.getPredictedPointInfo(timePred));
        timePred += 100;
    }

    comparePrediction_ = true;
}

void Physics::updateObstacles()
{
    int k = 0;
    for (auto i: agents_)
    {
        agentObstacles_[k]->setCenter(i->getPositionFiltered());
        k++;
    }

    k = 0;
    for (auto i: enemies_)
    {
        enemyObstacles_[k]->setCenter(i->GetPos());
        k++;
    }

    ballObstacle_->setCenter(ballPositionFiltered_);
}
























