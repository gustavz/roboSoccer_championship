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
    STOP_BALL,
	NEAR_GOAL,
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

	/**
	 *@brief set side of team
	 */
	virtual void setSide(eSide s) = 0;

    /**
     *@brief sets a TargetPoint
     */
    void setTargetPoint(const TargetPoint& tp);
    /**
     *@brief sets a TargetPoint
     */
    void setTargetPoint(const Position& tp);
    /**
     *@brief set TargetPoints
     */
    void setTargetPoints(const std::vector<TargetPoint>& tp);
    /**
     *@brief set TargetPoints
     */
    void setTargetPoints(const std::vector<Position>& tp);
    /**
     *@brief adds a TargetPoint
     */
    void addTargetPoint(const TargetPoint& tp);
    /**
     *@brief adds a TargetPoint
     */
    void addTargetPoint(const Position& tp);    
    /**
     *@brief adds TargetPoints
     */
    void addTargetPoints(const std::vector<TargetPoint>& tp);    
    /**
     *@brief adds TargetPoints
     */
    void addTargetPoints(const std::vector<Position>& tp);
    /**
     *@brief delete all TargetPoints
     */
    void deleteTargetPoints(){ targetPoints_.clear();}

    /**
     *@brief To check if robot has reached his TargetPoint.
     *@return bool: true: Robot has reached his final position, false: robot is still driving
     */
    bool isAtTarget() const;

	const double ROBOT_RADIUS = 0.047; /**< robot radius */

    /**
     *@brief Activates the drive function of the robot.
     */
    void activate(){ active_ = true; }
    /**
     *@brief Deactivates the drive function of the robot.
     */
    void deactivate(){ active_ = false; }

    //====stopBall=====//
    /**
     *@brief Robot stops the ball.
     */
    void startStopBall();

    //====clearBall=====//
    /**
     *@brief If the ball is between roboter and our goal: Roboter overtakes the ball with collision avoidence. After that the roboter drives with full speed to the ball position.
             Otherwise: Roboter drives with fullspeed to ball position. Is cleared when ball direction is in enemy goal direction.
     */
    void startClearBall();

    /**
     *@brief Starts shooting the ball in goal direction.
     */
    void startShootBall();

    /**
     *@brief Stop all actions of the robot.
     */
    void stopAllActions();

    /**
     *@brief passes the ball to a roboter of our team
     *@param agent: name of agent the ball shall be passed to
     */
    void startPassTo(Agent* agent);
    /**
     *@brief calculate a Ball to target line
     *@param target: Position to which you want to shoot
     */
    Line getBallTargetLine(Position target);

    /**
     *@brief sets a new drive speed
     *@param speed: speed with which the robot should drive
     */
    void setDesiredSpeed(double speed) {desiredSpeed_ = speed;}

    /**
     *@brief get the filtered position
     *@return filtered position of the robot
     */
    Vector2d getPositionFiltered() {return position_;}

    /**
     *@brief Shoot the ball in right direction
     */
    void shoot();

    friend class Debug;
protected:




	/**
	 *@brief calculate shoot position depending on ball, target and distance to ball
	 */
    TargetPoint calcShootPosition(Position ball, Position target, double distToBall = 0.1, bool brake = false);

	Timestamp lastPositionTimestamp_;/**< time stamp of last position measurement */

	QTime timerAgent_;/**< timeer for agent update */
	QTime timerUpdate_;/**< time update */
	double kp_ = 0.4;/**< proportional gain */
	double ki_ = 0.02;/**< integral gain */
	double kd_ = 0.02;/**< differential gain */
	double lastError = 0; /**< last deviation */
	double speedIntegrated_ = 0; /**< integrator counter */
	double lastSpeed_ = 0; /**< last speed of the robot */

	boost::mutex targetPointMutex_; /**< Mutex to lock targetPoints vector */

	Vector2d heading_; /**< heading of the robot */

	Vector2d lastPosition_; /**< last robot position */
	double robotSpeed_ = 0; /**< current robot speed */

	double desiredSpeed_ = 0.4; /**< desired robot speed */
	bool driveBackwards; /**< robot is driving backwards */
	bool active_; /**< robot cruise is active */

	eSide ourSide_; /**< our side */

	bool useCA_ = true; /**< use collision avoidance in general */


	//====Play the Ball=====//
	StateMachine shootBallSM;

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
     *@brief changes the targetpoint if it is in the enemypenaltyZone
    */
    void avoidPenaltyZone(TargetPoint* target);



    //====Playing Modes====//
    bool attackerModeActive_ = false;       /**< status Atacker Mode */
    bool defenderModeActive_ = false;       /**< status Defender Mode */
    bool shootBallActive_ = false;          /**< status shootBall */
    bool kickOffActive_ = false;            /**< status kickOff */
	bool penaltyModeActive_ = false;		/**< status penalty */

	Quadrangle* ownCornerBottomLeft_; /**< Pointer to own corner bottomleft */
	Quadrangle* ownCornerBottomRight_; /**< pointer to own corner bottomright */
	LineSegment* ownGoalSegment_; /**< Pointer to own goal segment */
	Quadrangle* ownPenaltyZone_; /**< Pointer to own penalty zone*/
	Quadrangle* ownFieldHalf_; /**< Pointer to own field half*/

	LineSegment* enemyGoalSegment_; /**< Pointer to enemy goal segment */
	Quadrangle* enemyPenaltyZone_; /**< Pointer to enemy penalty zone */
	Quadrangle* enemyFieldHalf_; /**< Pointer to enemy field half*/


	Quadrangle* leftSide_; /**< Pointer to lower half of the game field */
	Quadrangle* rightSide_; /**< Pointer to upper half of the game field */



};

#endif // AGENT_H
