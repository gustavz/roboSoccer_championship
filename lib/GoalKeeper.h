#ifndef GOALKEEPER_H
#define GOALKEEPER_H

#include "Agent.h"
#include "Physics.h"
#include "RunnableObject.h"
#include "FieldPlayer.h"

namespace GOALKEEPER_STATES
{
    enum GoalyState
    {
        AUTO_HOLD_NOT_ACTIVE,
        AUTO_HOLD_ACTIVE,
        CLEAR_BALL
    };
}

namespace GOALKEEPER_KICK_STATES
{
    enum GkKickState
    {
        PREPARE,
        SHOOT
    };
}

class GoalKeeper : public Agent
{
public:
    /**
     *@brief constructor
     *@param DBC: connection to camera
     *@param deviceNr: number of roboter
     *@param physics: pointer to physics
     */
    GoalKeeper(RTDBConn& DBC, const int deviceNr, Physics* physics);

    /**
     *@brief default destructor
     */
    ~GoalKeeper();

    /**
     *@brief set side of goalkeeper's goal
     *@param s: side of our goal
     */
    void setSide(eSide s);

    /**
     *@brief starts goalkeeper
     */
    void startGoalKeeper();

    /**
     *@brief stops goalkeeper
     */
    void stopGoalKeeper();

    /**
     *@brief runs state machine
     */
    void run();

    /**
     *@brief set the first defender
     */
    void setFirstDefender(FieldPlayer* fp){firstDefender_ = fp;}

    /**
     *@brief set the second defender, setting attacker to null
     */
    void setSecondDefender(FieldPlayer* fp){secondDefender_ = fp; attacker_ = NULL;}

    /**
     *@brief set attacker, setting defender to null
     */
    void setAttacker(FieldPlayer* fp){attacker_ = fp; secondDefender_ = NULL;}

private:

    /**
     *@brief follows the ball
     */
    void followBall();

    /**
     *@brief anticipates the ball trajectory
     */
    void anticipateBall();

    /**
     *@brief determines if projected ball trajectory reaches the goal
     */
    bool trajectoryOnGoal();

    /**
     *@brief Determines if ball is inside of his penalty area
     */
    bool shouldClearBall();

    /**
     *@brief perform goalkeeper kick
     */
    void gkKick();

    /**
     *@brief calculate the gk kick position behind the ball
     */
    void calcPrepareGkKick();

    Physics* physics_;      /**< pointer to physics */

    bool goalKeeperActive_; /**< status that indicates if goalkeeper is active */

    FieldPlayer* firstDefender_; /**< pointer to first defender*/

    FieldPlayer* secondDefender_; /**< pointer to second defender if exists*/

    FieldPlayer* attacker_; /**< pointer to attacker if exists*/

    LineSegment protectionSegment_; /**< Segment on which goalkeeper will keep defending the goal*/

    const int anticipationTime_ = 5000; /**< Time in ms how far to predict the ball trajectory*/

    LineSegment* ownInnerGoal_;

    StateMachine gkKickSM; /**< StateMachine for the Goalkeeper kick */
    Vector2d lastBallPosBeforeKick_; /**< Last measured ball position before gk kick */


};

#endif // GOALKEEPER_H


