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

    //====Block Enemy=====//
    Enemy* enemyToBlock_;
    double blockingDistToBall_;

    /**
     *@brief check weather it is possible to block an enemy and initialize values for blocking
     */
    bool startBlockEnemy();

    /**
     *@brief blockes the enemy from ball, allowing others to clear the ball
     */
    void blockEnemy();



    //====Anticiapte====//
    void anticipatePass();
    void prepareShoot();

    //====AttackerMode====//
    StateMachine attackerModeSM;
    void attackerMode();

    //====DefenderMode====//
    StateMachine defenderModeSM;
    void defenderMode();
    DefenderRole defenderRole_;

    //====kickOff====//
    StateMachine kickOffSM;
    void kickOff();
    void calcKickOffPreparePosition();






};

#endif // FIELDPLAYER_H


