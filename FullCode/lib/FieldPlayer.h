#ifndef FIELDPLAYER_H
#define FIELDPLAYER_H

#include "Agent.h"
#include "RunnableObject.h"
#include "Physics.h"

enum DefenderRole
{
    DEFEND_FRONT,
    DEFEND_BACK,
    DEFEND_ALONE
};

namespace ATTACKER_STATES
{
    enum AttackerStates
    {
        ANTICIPATE,
        SHOOT
    };
}

namespace DEFENDER_STATES
{
    enum DefenderStates
    {
        SUPPORT_GK,
        CLEAR_BALL,
        PASS_BALL,
        SHOOT_ON_GOAL,
        BLOCK_ENEMY,
        MOVE_ASIDE
    };
}

namespace KICKOFF_STATES
{
enum ClearBallStates
{
    PREPARE,
    PREPARE_KICKOFF,
    SHOOT
};
}


class FieldPlayer : public Agent
{
public:

    /**
     *@brief constructor for class FieldPlayer
     *@param DBC: connection to camera
     *@param deviceNr: number of robot
     *@param physics: pointer to physics class
     */
    FieldPlayer(RTDBConn& DBC, const int deviceNr, Physics* physics);

    /**
     *@brief default destructor
     */
    ~FieldPlayer();

    /**
     *@brief runs the state machine
     */
    void run();

    /**
     *@brief sets the side of our team
     *@param s: our side
     */
    void setSide(eSide s);

    /**
     *@brief starts attacker mode
     */
    void startAttackerMode();

    /**
     *@brief starts defender mode
     */
    void startDefenderMode(DefenderRole role);

    /**
     *@brief starts KickOff
     */
    void startKickOff();


    /**
     *@brief set defender, setting attacker to null
     */
    void setDefender(FieldPlayer* fp){defender_ = fp; attacker_ = NULL;}

    /**
     *@brief set attacker, setting defender to null
     */
    void setAttacker(FieldPlayer* fp){attacker_ = fp; defender_ = NULL;}

    /**
     *@brief set a target point for kickoff preparation
     */
    void setKickoffPreparationTarget(TargetPoint tp){setTargetPoint(tp);}


private:
    FieldPlayer* defender_; /**< pointer to defender if exists*/
    FieldPlayer* attacker_; /**< pointer to attacker if exists*/

    //====Support Goaly====//
    LineSegment defenseLine_;   /**< line segment the field player moves on */

    /**
     *@brief calculate the defenseLine for gk support
	 */
    void startSupportGk(double distToHalfLine);

    /**
     *@brief supports goal keeper by trying to block the ball
     */
    void supportGk();


    //====AttackerMode====//
	StateMachine attackerModeSM; /**< statemachine for the attacker mode*/
	/**
	  *@brief Run the attackerModeSM
	  */
    void attackerMode();

    //====DefenderMode====//
	StateMachine defenderModeSM; /**< statemachine for the defender mode*/
	/**
	  *@brief Run the defenderModeSM
	  */
	void defenderMode();
	DefenderRole defenderRole_; /**< role of defender*/

    //====kickOff====//
	StateMachine kickOffSM; /**< statemachine for the kickoff */
	/**
	  *@brief Run the kickOffSM
	  */
    void kickOff();
	/**
	  *@brief Calculate the position to shoot
	  */
    void calcKickOffPreparePosition();


};

#endif // FIELDPLAYER_H


