#ifndef PHYSICS_H
#define PHYSICS_H

#include "Agent.h"
#include "Enemy.h"
#include "Ball.h"
#include "RunnableObject.h"
#include "Geometry.h"
#include "QTime"
#include "helper/Trajectory.h"

#include <boost/smart_ptr/shared_ptr.hpp>

#include <vector>

class Physics : public RunnableObject
{
public:

    struct GameField
    {
        Quadrangle Field;
        LineSegment GoalLeft;
        LineSegment GoalRight;
        LineSegment InnerGoalLeft;
        LineSegment InnerGoalRight;
        LineSegment HalfwayLine;
        Quadrangle PenaltyAreaLeft;
        Quadrangle PenaltyAreaRight;
        Quadrangle ObstaclePenaltyAreaLeft;
        Quadrangle ObstaclePenaltyAreaRight;
        Quadrangle ObstacleField;
        Quadrangle CornerBottomLeft;
        Quadrangle CornerBottomRight;
        Quadrangle CornerTopRight;
        Quadrangle CornerTopLeft;
        Quadrangle ObstacleCornerBottomLeft;
        Quadrangle ObstacleCornerBottomRight;
        Quadrangle ObstacleCornerTopRight;
        Quadrangle ObstacleCornerTopLeft;
        Quadrangle LeftHalf;
        Quadrangle RightHalf;
        Quadrangle UpperHalf;
        Quadrangle LowerHalf;

        GameField(Quadrangle field, LineSegment goalLeft, LineSegment goalRight, LineSegment innerGoalLeft,
                  LineSegment innerGoalRight, LineSegment halfwayLine, Quadrangle penaltyLeft,
                  Quadrangle penaltyRight, Quadrangle obstaclePenaltyLeft, Quadrangle obstaclePenaltyRight,
                  Quadrangle obstacleField, Quadrangle cornerBottomLeft, Quadrangle cornerBottomRight,
                  Quadrangle cornerTopRight, Quadrangle cornerTopLeft,
                  Quadrangle obstacleCornerBottomLeft, Quadrangle obstacleCornerBottomRight,
                  Quadrangle obstacleCornerTopRight, Quadrangle obstacleCornerTopLeft,
                  Quadrangle leftHalf, Quadrangle rightHalf, Quadrangle upperHalf, Quadrangle lowerHalf)
            :
              Field(field),
              GoalLeft(goalLeft),
              GoalRight(goalRight),
              InnerGoalLeft(innerGoalLeft),
              InnerGoalRight(innerGoalRight),
              HalfwayLine(halfwayLine),
              PenaltyAreaLeft(penaltyLeft),
              PenaltyAreaRight(penaltyRight),
              ObstaclePenaltyAreaLeft(obstaclePenaltyLeft),
              ObstaclePenaltyAreaRight(obstaclePenaltyRight),
              ObstacleField(obstacleField),
              CornerBottomLeft(cornerBottomLeft),
              CornerBottomRight(cornerBottomRight),
              CornerTopRight(cornerTopRight),
              CornerTopLeft(cornerTopLeft),
              ObstacleCornerBottomLeft(obstacleCornerBottomLeft),
              ObstacleCornerBottomRight(obstacleCornerBottomRight),
              ObstacleCornerTopRight(obstacleCornerTopRight),
              ObstacleCornerTopLeft(obstacleCornerTopLeft),
              LeftHalf(leftHalf),
              RightHalf(rightHalf),
              UpperHalf(upperHalf),
              LowerHalf(lowerHalf)

        {
        }
    };

    /**
     *@brief default contructor
    */
    Physics();
    /**
     *@brief copy constructor
     *@param ball: initializes member ball_
    */
    Physics(Ball* ball);

    /**
     *@brief initializes obstacles
    */
    void initializePhysics();
    /**
     *@brief adds Agent
     *@param ag: Agent to be added
    */
    void addAgent(Agent* ag);
    /**
     *@brief adds Enemy
     *@param em: Enemy to be added
    */
    void addEnemy(Enemy* em);

    /**
     *@brief returns enemy specified by argument
     *@param nr: id of enemy
    */
    Enemy* getEnemy(int nr){return enemies_[nr];}
    /**
     *@brief returns agent specified by argument
     *@param nr: id of agent
    */
    Agent* getAgent(int nr){return agents_[nr];}
    /**
     *@brief returns ball
    */
    Ball* getBall(){return ball_;}
    /**
     *@brief returns all enemies as vector
    */
    std::vector<Enemy*> getEnemies(){return enemies_;}

    /**
     *@brief returns number of agents
    */
    int getNumberOfAgents() const {return agents_.size();}
    /**
     *@brief returns number of enemies
    */
    int getNumberOfEnemies() const {return enemies_.size();}
    /**
     *@brief returns number of robots in total
    */
    int getNumberOfPlayers() const {return agents_.size() + enemies_.size();}

    /**
     *@brief checks if position is inside field
     *@param pos: position to be checked
    */
    bool isInsideGameField(Position const& pos) const {return gameField_.Field.isInside(pos);}
    /**
     *@brief checks if position is inside left penalty area
     *@param pos: position to be checked
    */
    bool isInsideLeftPenaltyArea(Position const& pos) const {return gameField_.PenaltyAreaLeft.isInside(pos);}
    /**
     *@brief checks if position is inside right penalty area
     *@param pos: position to be checked
    */
    bool isInsideRightPenaltyArea(Position const& pos) const {return gameField_.PenaltyAreaRight.isInside(pos);}
    /**
     *@brief checks if position is inside any penalty area
     *@param pos: position to be checked
    */
    bool isInsidePenaltyArea(Position const& pos) const {return isInsideLeftPenaltyArea(pos) || isInsideRightPenaltyArea(pos);}

    /**
     *@brief checks if position is inside left half
     *@param pos: position to be checked
    */
    bool isInsideLeftHalf(Position const& pos) const {return gameField_.LeftHalf.isInside(pos);}
    /**
     *@brief checks if position is inside right half
     *@param pos: position to be checked
    */
    bool isInsideRightHalf(Position const& pos) const {return gameField_.RightHalf.isInside(pos);}
    /**
     *@brief checks if position is inside upper half
     *@param pos: position to be checked
    */
    bool isInsideUpperHalf(Position const& pos) const {return gameField_.UpperHalf.isInside(pos);}
    /**
     *@brief checks if position is inside lower half
     *@param pos: position to be checked
    */
    bool isInsideLowerHalf(Position const& pos) const {return gameField_.LowerHalf.isInside(pos);}

    /**
     *@brief returns left goal line
    */
    LineSegment getGoalLeft() const {return gameField_.GoalLeft;}
    /**
     *@brief returns right goal line
    */
    LineSegment getGoalRight() const {return gameField_.GoalRight;}
    LineSegment getInnerGoalLeft() const {return gameField_.InnerGoalLeft;}
    LineSegment getInnerGoalRight() const {return gameField_.InnerGoalRight;}
    LineSegment getHalfwayLine() const {return gameField_.HalfwayLine;}

    /**
     *@brief returns left penalty area
    */
    Quadrangle getPenaltyAreaLeft() const {return gameField_.PenaltyAreaLeft;}
    /**
     *@brief returns right penalty area
    */
    Quadrangle getPenaltyAreaRight() const {return gameField_.PenaltyAreaRight;}

    /**
     *@brief returns left penalty area with collision avoidance margin
    */
    Quadrangle getObstaclePenaltyAreaLeft() const {return gameField_.ObstaclePenaltyAreaLeft;}
    /**
     *@brief returns right penalty area with collision avoidance margin
    */
    Quadrangle getObstaclePenaltyAreaRight() const {return gameField_.ObstaclePenaltyAreaRight;}

    /**
     *@brief returns game field
    */
    Quadrangle getField() const {return gameField_.Field;}

    /**
     *@brief returns game field with collision avoidance margin
    */
    Quadrangle getObstacleField() const {return gameField_.ObstacleField;}

    /**
     *@brief returns bottom left corner
    */
    Quadrangle getCornerBottomLeft() const {return gameField_.CornerBottomLeft;}
    /**
     *@brief returns bottom right corner
    */
    Quadrangle getCornerBottomRight() const {return gameField_.CornerBottomRight;}
    /**
     *@brief returns top right corner
    */
    Quadrangle getCornerTopRight() const {return gameField_.CornerTopRight;}
    /**
     *@brief returns top left corner
    */
    Quadrangle getCornerTopLeft() const {return gameField_.CornerTopLeft;}
    /**
     *@brief returns bottom left corner with collision avoidance margin
    */
    Quadrangle getObstacleCornerBottomLeft() const {return gameField_.ObstacleCornerBottomLeft;}
    /**
     *@brief returns bottom right corner with collision avoidance margin
    */
    Quadrangle getObstacleCornerBottomRight() const {return gameField_.ObstacleCornerBottomRight;}
    /**
     *@brief returns top right corner with collision avoidance margin
    */
    Quadrangle getObstacleCornerTopRight() const {return gameField_.ObstacleCornerTopRight;}
    /**
     *@brief returns top left corner with collision avoidance margin
    */
    Quadrangle getObstacleCornerTopLeft() const {return gameField_.ObstacleCornerTopLeft;}

    /**
     *@brief returns left half of field
    */
    Quadrangle getLeftHalf() const {return gameField_.LeftHalf;}
    /**
     *@brief returns right half of field
    */
    Quadrangle getRightHalf() const {return gameField_.RightHalf;}
    /**
     *@brief returns upper half of field
    */
    Quadrangle getUpperHalf() const {return gameField_.UpperHalf;}
    /**
     *@brief returns lower half of field
    */
    Quadrangle getLowerHalf() const {return gameField_.LowerHalf;}

    /**
     *@brief returns pointer to left goal line
    */
    LineSegment* getGoalLeftPtr() {return &gameField_.GoalLeft;}
    LineSegment* getGoalRightPtr() {return &gameField_.GoalRight;}    
    LineSegment* getInnerGoalLeftPtr() {return &gameField_.InnerGoalLeft;}
    LineSegment* getInnerGoalRightPtr() {return &gameField_.InnerGoalRight;}
    /**
     *@brief returns pointer to middle line
    */
    LineSegment* getHalfwayLinePtr() {return &gameField_.HalfwayLine;}

    /**
     *@brief returns pointer to left penalty area with collision avoidance margin
    */
    Quadrangle* getObstaclePenaltyAreaLeftPtr() {return &gameField_.ObstaclePenaltyAreaLeft;}
    /**
     *@brief returns pointer to right penalty area with collision avoidance margin
    */
    Quadrangle* getObstaclePenaltyAreaRightPtr() {return &gameField_.ObstaclePenaltyAreaRight;}

    /**
     *@brief returns pointer to game field
    */
    Quadrangle* getFieldPtr() {return &gameField_.Field;}
    /**
     *@brief returns pointer to game field with collision avoidance margin
    */
    Quadrangle* getObstacleFieldPtr() {return &gameField_.ObstacleField;}

    /**
     *@brief returns pointer to bottom left corner
    */
    Quadrangle* getCornerBottomLeftPtr() {return &gameField_.CornerBottomLeft;}
    /**
     *@brief returns pointer to bottom right corner
    */
    Quadrangle* getCornerBottomRightPtr() {return &gameField_.CornerBottomRight;}
    /**
     *@brief returns pointer to top right corner
    */
    Quadrangle* getCornerTopRightPtr() {return &gameField_.CornerTopRight;}
    /**
     *@brief returns pointer to top left corner
    */
    Quadrangle* getCornerTopLeftPtr() {return &gameField_.CornerTopLeft;}
    /**
     *@brief returns pointer to bottom left corner with collision avoidance margin
    */
    Quadrangle* getObstacleCornerBottomLeftPtr() {return &gameField_.ObstacleCornerBottomLeft;}
    /**
     *@brief returns pointer to bottom right corner with collision avoidance margin
    */
    Quadrangle* getObstacleCornerBottomRightPtr() {return &gameField_.ObstacleCornerBottomRight;}
    /**
     *@brief returns pointer to top right corner with collision avoidance margin
    */
    Quadrangle* getObstacleCornerTopRightPtr() {return &gameField_.ObstacleCornerTopRight;}
    /**
     *@brief returns pointer to top left corner with collision avoidance margin
    */
    Quadrangle* getObstacleCornerTopLeftPtr() {return &gameField_.ObstacleCornerTopLeft;}

    /**
     *@brief returns pointer to left half of field
    */
    Quadrangle* getLeftHalfPtr() {return &gameField_.LeftHalf;}
    /**
     *@brief returns pointer to right half of field
    */
    Quadrangle* getRightHalfPtr() {return &gameField_.RightHalf;}
    /**
     *@brief returns pointer to upper half of field
    */
    Quadrangle* getUpperHalfPtr() {return &gameField_.UpperHalf;}
    /**
     *@brief returns pointer to lower half of field
    */
    Quadrangle* getLowerHalfPtr() {return &gameField_.LowerHalf;}


    /**
     *@brief returns ball with collision avoidance margin
    */
    Obstacle* getBallObstacle() {return ballObstacle_;}
    /**
     *@brief returns all agents with collision avoidance margin
    */
    std::vector<Obstacle*> getAgentObstacles() {return agentObstacles_;}
    /**
     *@brief returns all enemies with collision avoidance margin
    */
    std::vector<Obstacle*> getEnemyObstacles() {return enemyObstacles_;}

    /**
     *@brief returns filtered ball position
    */
    Vector2d getBallPositionFiltered() { return ballPositionFiltered_; }
    /**
     *@brief returns last ball position
    */
    Position getBallLastPosition() { return lastBallPosition_.toPosition(); }
    /**
     *@brief returns predicted ball position
     *@param milliseconds: prediction time
    */
    Position getPredBallPosition(int milliseconds) const;
    /**
     *@brief returns predicted ball velocity
     *@param milliseconds: prediction time
    */
    Vector2d getPredBallVelocity(int milliseconds) const;
    /**
     *@brief returns current ball velocity
    */
    Vector2d getBallVelocity() const;
    /**
     *@brief returns simple ball trajectory
    */
    Line getSimpleBallTrajectory() const;
    /**
     *@brief returns predicted ball trajectory
     *@param milliseconds: prediction time
    */
    std::vector<LineSegment> getBallTrajectory(int milliseconds) const;

    /**
     *@brief returns enemy closest to ball
    */
    Enemy* getEnemyClosestToBall();
    /**
     *@brief returns distance to closest enemy from argument position
     *@param vec: position to be checked
    */
    double getClosestEnemysDistance(const Vector2d& vec) const;
    /**
     *@brief returns distance to ball of enemy closest to ball
    */
    double getClosestEnemysDistanceToBall() const;

    /**
     *@brief main update routine of Physics
    */
    void run();

    const double ROBOT_RADIUS = 0.047; /**< physical robot radius*/
    const double ROBOT_OBSTACLE_RADIUS = 3.5 * ROBOT_RADIUS; /**< robot radius with collision avoidance margin*/
    const double BALL_RADIUS = 0.021335; /**< physical ball radius*/
    const double BALL_OBSTACLE_RADIUS = BALL_RADIUS + 1.5 * ROBOT_RADIUS; /**< ball radius with collision avoidance margin*/
    const double PENALTY_AREA_MARGIN = 1.1 * ROBOT_RADIUS; /**< collision avoidance margin for penalty areas*/
    const double FIELD_MARGIN = 1.5 * ROBOT_RADIUS; /**< collision avoidance margin for field */
    const double CORNER_MARGIN = 1.1 * ROBOT_RADIUS; /**< collision avoidance margin for corners*/


    void startComparePrediction(int predictionTime);
    void stopComparePrediction(){comparePrediction_ = false;}


private:

    QTime velTimer_;

    void updatePhysics();

    void updateObstacles();

    void PredictBallTrajectory(int milliseconds);

    Vector2d ballVelocity_;

    std::vector<Agent* > agents_;
    std::vector<Enemy* > enemies_;

    std::vector<Obstacle* > agentObstacles_;
    std::vector<Obstacle* > enemyObstacles_;

    Ball* ball_;
    Obstacle* ballObstacle_;


    Vector2d lastBallPosition_;
    Vector2d ballPositionFiltered_;

    GameField gameField_;

    Trajectory ballTrajectory_;

    Line simpleBallTrajectory_;


    bool comparePrediction_ = false;
    std::vector<PointInfo> predictions_;
};


/**
 * @brief Performs a moving average filter on the value
 * @param value: Value to be filtered
 * @param updatedValue: new Value
 * @param factor: Factor for the new Value
 */
template <typename T>
void movingAverage(T& value, T updatedValue, double factor = 0.8)
{
    value = value * (1 - factor) + (updatedValue * factor);
}

/**
 * @brief Template to constrain a value. Enter min value first!!!
 * @param val: Value to be constraint
 * @param minVal: minimum Value
 * @param maxVal: maximum Value
 */
template <typename T>
void constraint(T& val, T minVal, T maxVal)
{
    assert(minVal <= maxVal);
    if (val < minVal)
    {
        val = minVal;
    }
    else if (val > maxVal)
    {
        val = maxVal;
    }
}

#endif // PHYSICS_H
