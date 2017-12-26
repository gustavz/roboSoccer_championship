#ifndef ENEMY_H
#define ENEMY_H

#include "robo_control.h"

class Physics;

class Enemy : public RoboControl
{
  public:
    /**
     *@brief constructor for class Enemy
     *@param DBC: connection to camera
     *@param deviceNr: number of robot
     */
    Enemy(RTDBConn& DBC, const int deviceNr, Physics* physics)
        :
          RoboControl(DBC, deviceNr),
          physics_(physics)
    {
    }

    /**
     *@brief default destructor
     */
    ~Enemy() {}

    double getDistToBall();


private:

    Physics* physics_;

};

#endif // ENEMY_H
