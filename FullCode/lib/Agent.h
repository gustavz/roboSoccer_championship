#ifndef AGENT_H
#define AGENT_H

#include "robo_control.h"
#include <vector>
#include "Path.h"
#include "RunnableObject.h"
#include <boost/optional.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include "StateMachine.h"

#include <QTime>

class Physics;

namespace STOPBALL_STATES
{
enum StopBallStates
{
    INIT,
    NOT_MOVING_BALL,
    BEFORE_BALL,
    OVERTAKE_BALL,
    BLOCK_BALL,
    END
};
}

namespace CLEARBALL_STATES
{
enum ClearBallStates
{
    INIT,
    IS_IN_BALL_DIRECTION,
    IS_NOT_IN_BALL_DIRECTION,
    STOP_BALL,
    CLEAR,
    END
};
}

namespace SHOOTBALL_STATES
{
enum ShootBallStates
{
    INIT,
    GET_BEHIND_BALL,
    GET_ON_BALL_GOAL_LINE,
    SHOOT_BALL,
    KICK,
    END
};
}

namespace PASSTO_STATES
{
enum PassToStates
{
    INIT,
    GET_BEHIND_BALL,
    GET_ON_BALL_TARGET_LINE,
    PASS_BALL,
    END
};
}

namespace SUPPORT_GK_STATES
{
enum SupportGkStates
{
    SHORTEN_ANGLE,
    BLOCK,
    MOVE_AWAY
};
}

class Agent :   public RoboControl,
                public Path,
                public RunnableObject
{
public:
    /**
     *@brief constructor for class Agent
     *@param DBC: connection to camera
     *@param deviceNr: number of robot
     *@param physics: pointer to physics class
     *@param initState: initial state
     *@param interval: calculation interval
     */
    Agent(RTDBConn& DBC, const int deviceNr, Physics* physics, int initState, int interval);

    /**
     *@brief default destructor
     */
    ~Agent();

    /**
     *@brief runs state machine
     */
    virtual void run() = 0;
    /**
     *@brief updates Timestamp of all agent functions
     */
    void update();
    /**
     *@brief moves the robot to its TargetPoint
     */
    void cruise();
    /**
     *@brief activates Collisionavoidance for different params
     *@param activates enemies
     *@param activates agents
     *@param activates ownPenaltyZone
     *@param activates enemyPenaltyZone
     *@param activates ball
     *@param activates gameField
     */
    void activateCA(bool enemies, bool agents, bool ownPenaltyZone, bool enemyPenaltyZone, bool ballObst, bool gameField);
    /**
     *@brief deactivates Collisionavoidance
     */
    void deactivateCA() {useCA_ = false;}
    /**
     *@brief turns the robot to a given Vector or Targetpoint
     */
    bool turn(const TargetPoint& tp, bool precise = false);
    bool turn(const Vector2d& dir, double precision);
    void turn(const Vector2d& dir, bool precise = false);
    bool getDriveBackwards() { return driveBackwards; }

    virtual void setSide(eSide s) = 0;

    /**
     *@brief sets a TargetPoint
     */
    void setTargetPoint(const TargetPoint& tp);
    void setTargetPoint(const Position& tp);
    void setTargetPoints(const std::vector<TargetPoint>& tp);
    void setTargetPoints(const std::vector<Position>& tp);
    void addTargetPoint(const TargetPoint& tp);
    /**
     *@brief adds a TargetPoint
     */
    void addTargetPoint(const Position& tp);    
    void addTargetPoints(const std::vector<TargetPoint>& tp);    
    void addTargetPoints(const std::vector<Position>& tp);
    void deleteTargetPoints(){ targetPoints_.clear();}

    bool isAtTarget() const;

    const double ROBOT_RADIUS = 0.047;

    void activate(){ active_ = true; }
    void deactivate(){ active_ = false; }

    //====stopBall=====//
    void startStopBall();

    //====clearBall=====//
    /**
     *@brief If the ball is between roboter and our goal: Roboter overtakes the ball with collision avoidence. After that the roboter drives with full speed to the ball position.
             Otherwise: Roboter drives with fullspeed to ball position. Is cleared when ball direction is in enemy goal direction.
     */
    void startClearBall();

    void startShootBall();

    void stopAllActions();

    /**
     *@brief passes the ball to a roboter of our team
     *@param agent: name of agent the ball shall be passed to
     */
    void startPassTo(Agent* agent);
    Line getBallTargetLine(Position target);

    void setDesiredSpeed(double speed) {desiredSpeed_ = speed;}

    Vector2d getPositionFiltered() {return position_;}


	/**
	 *@brief Shoot the ball in right direction
	 */
	void shoot();

    friend class Debug;
protected:
    Vector2d calcRelativePosition(double x, double distance_left_or_right, eSide left_or_right);
    LineSegment calcPositionBeforeBall(Position ball, Position target, double distToBall = 0.1);

    TargetPoint calcShootPosition(Position ball, Position target, double distToBall = 0.1, bool brake = false);

    Timestamp lastPositionTimestamp_;

    QTime timerAgent_;
    QTime timerUpdate_;
    double kp_ = 0.4;
    double ki_ = 0.02;
    double kd_ = 0.02;
    double lastError = 0;
    double speedIntegrated_ = 0;
    double lastSpeed_ = 0;

    boost::mutex targetPointMutex_;

    Vector2d heading_;

    Vector2d lastPosition_;
    double robotSpeed_ = 0;

    double desiredSpeed_ = 0.4;
    bool driveBackwards;
    bool active_;

    eSide ourSide_;

    bool useCA_ = true;

    //====stopBall=====//
    StateMachine stopBallSM;
    /**
     *@brief If the roboter is behind the ball: Roboter overtakes the ball with collision avoidence. After that the roboter stops in front of the ball.
             Otherwise: Roboter drives to ball position and stops.
     */
    void stopBall();
    /**
     *@brief Calculates the scalar product of ball-direction-vector and roboter to ball vector.
      @param val:Value for the decision logic it represents the angle.
      @return True if scalar-product-value < val
     */
    bool isBeforBall(double val);

    //====clearBall=====//
    StateMachine clearBallSM;
    void clearBall();


    //====Play the Ball=====//
    StateMachine shootBallSM;
    StateMachine passToSM;
    Agent* passToAgent_;

    /**
     *@brief moves behind ball and shoots it at the enemy goal
    */
    void shootBall();
    /**
     *@brief checks if the robot is behind the ball
      @param val: Distance he should be behind the ball
    */
    bool isBehindBall(double val = 0);
    /**
     *@brief checks if the robot is in a triangle pointing to the goal behind the ball
    */
    bool isGoodBehindBall();
    /**
     *@brief calculates the time the robot needs to get to the ball
    */
    double timeToBall();
    /**
     *@brief creates a Line through the enemy goal middlepoint and the ball position
    */
    Line getBallGoalLine();
    /**
     *@brief moves behind the ball and passes it to a friendly fieldplayer
    */
    void passTo();
    /**
     *@brief changes the targetpoint if it is in the enemypenaltyZone
    */
    void avoidPenaltyZone(TargetPoint* target);



    //====Playing Modes====//
    bool attackerModeActive_ = false;       /**< status Atacker Mode */
    bool defenderModeActive_ = false;       /**< status Defender Mode */
    bool supportGkActive_ = false;          /**< status support goalkeeper */
    bool shootBallActive_ = false;          /**< status shootBall */
    bool clearBallActive_ = false;          /**< status clearBall */
    bool stopBallActive_ = false;           /**< status stopBall */
    bool passToActive_ = false;             /**< status passTo */
    bool kickOffActive_ = false;            /**< status kickOff */

    // relative Field
    Quadrangle* ownCornerBottomLeft_;
    Quadrangle* ownCornerBottomRight_;
    LineSegment* ownGoalSegment_;
    Quadrangle* ownPenaltyZone_;
    Quadrangle* ownFieldHalf_;

    LineSegment* enemyGoalSegment_;
    Quadrangle* enemyPenaltyZone_;
    Quadrangle* enemyFieldHalf_;

    // Aus Sicht von den PCs auf das Spielfeld.
    Quadrangle* leftSide_;
    Quadrangle* rightSide_;


};

#endif // AGENT_H
