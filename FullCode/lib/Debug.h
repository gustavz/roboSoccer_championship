#ifndef DEBUG_H
#define DEBUG_H

#include "GameControl.h"
#include "RunnableObject.h"

namespace DEBUG_STATES{
enum DebugStates
{
    NO_DEBUG,
    CRUISE,
    INTERCEPT,
    PENALTY,
    START,
    SHOOT,
    PREDICTION,
    REFEREE
};
}

class Debug : public RunnableObject
{
public:

    /**
     *@brief constructor for Debug class
     *@param gameControl: pointer to game control
     */
    Debug(GameControl* gameControl);

    /**
     *@brief runs state machine
     */
    void run();

    /**
     *@brief allows GameControl to access private fields of Debug class
     */
    friend class GameControl;

private:
  GameControl* gameControl_;

  void updateRef(std::string message);

};

#endif // DEBUG_H
