#ifndef GAMECONTROL_H
#define GAMECONTROL_H

#include "referee.h"
#include "Physics.h"
#include "GoalKeeper.h"
#include "FieldPlayer.h"
#include "thread"
//#include "Debug.h"

namespace PENALTY_STATES
{
enum PenaltyStates
{
    INIT,
    PREPARE_KICK,
    KICK,
    PASSIVE,
    END
};
}

namespace GAMECONTROL_STATES
{
enum GameControlStates
{
    REFEREE_INIT,
    BEFORE_KICK_OFF,
    KICK_OFF,
    BEFORE_PENALTY,
    PENALTY,
    PLAY_ON,
    PAUSE,
    TIME_OVER,
    DEBUG_CRUISE,
    DEBUG_INTERCEPT,
    DEBUG_SHOOT,
    DEBUG_PASSTO,
    ATTACKER_MODE,
    DEFENDER_MODE
};
}

namespace STRATEGIES
{
enum Strategies
{
    OFFENSIVE,
    DEFENSIVE
};
}

namespace TACTIC_STATES
{
enum TacticStates
{
    INIT,
    OFFENSE_ENEMY_SIDE,
    OFFENSE_OUR_SIDE,
    DEFENSE_ENEMY_SIDE,
    DEFENSE_OUR_SIDE

};
}


class GameControl : public RunnableObject
{
public:
    /**
     *@brief constructor
     *@param ref: pointer to referee
     *@param physics: pointer to physics
     *@param colorT: colour of our team
     */
    GameControl(Referee* ref, Physics* physics, eTeam colorT);

    /**
     *@brief runs state machine
     */
    virtual void run();

    /**
     *@brief sets goalkeeper as class member
     *@param gk: pointer to goalkeeper
     */
    void setGoalKeeper(GoalKeeper* gk){gk_ = gk;}

    /**
     *@brief sets field player 1 as class member
     *@param fp1: pointer to field player 1
     */
    void setFieldPlayer1(FieldPlayer* fp1){fp1_ = fp1;}

    /**
     *@brief sets field player 2 as class member
     *@param fp2: pointer to field player 2
     */
    void setFieldPlayer2(FieldPlayer* fp2){fp2_ = fp2;}

    /**
     *@brief chooses permanent predefined strategy
     */
    void setPermanentStrategy(STRATEGIES::Strategies strategy);

    friend class Debug; /**< allows Debug to access private fields of GameControl */
    friend class MainWindow;
private:


    /**
     *@brief updates referee information and indicates if game state has changed
     */
    bool updateRef();

    TargetPoint prepareShoot(Position ball, Position target, double distToBall);

    //========PLAY_ON====================
    bool autoStrategy_ = true; /**< automatically updated strategy active */
    STRATEGIES::Strategies strategy_;   /**< permanent state of strategy state machine */

    /**
     *@brief chooses strategy based on initial input or current score
     */
    void chooseStrategy();

    /**
     *@brief performs actions according to tactic
     */
    void playOn();


    //========PENALTY====================


    StateMachine penaltySM; /**< penalty state machine*/

    /**
     *@brief prepares for the penalty
     */
    void beforePenalty();

    /**
     *@brief performs the penalty
     */
    void penalty();

    //========KICKOFF====================
    /**
     *@brief prepares the kickoff
     */
    void beforeKickOff();

    /**
     *@brief performs kickoff actions
     */
    void kickOff();

    //=============NOCH NOTWENDIG?========
    Position getInterceptionParallelTarget(int leftOfBall) const;


    Referee* ref_;      /**< pointer to referee */
    Physics* physics_;  /**< pointer to physics */
    int playmode_;      /**< state of play mode */
    int lastPlaymode_;  /**< last state of play mode */
    eSide kickoff_;     /**< kickoff side */
    eSide ourSide_;     /**< our side */
    eSide kickOffSide_; /**< */

    eSide penaltyTeamShoot_;

    eTeam ourTeam_;     /**< our team */
    GoalKeeper* gk_;    /**< our goalkeeper */
    FieldPlayer* fp1_;  /**< our first field player */
    FieldPlayer* fp2_;  /**< our second field player */

    bool debugActive_ = false;      /**< status if debug mode is active */
    double debugCruiseSpeed_ = 1.;  /**< cruise speed in debug mode */


    Position ca1_1;
    Position ca1_2;
    Position ca1_3;
    Position ca1_4;

    bool measuringActive_ = false;




//    thread t1(&gk_::start, gk_);

};

#endif // GAMECONTROL_H
