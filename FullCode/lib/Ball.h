#ifndef BALL_H
#define BALL_H

#include "raw_ball.h"

class Ball : public RawBall
{

public:

    Ball(RTDBConn &DBC) : RawBall(DBC) {}

    Timestamp getLastTimestamp()
    {
        return mBall.timestamp;
    }


};

#endif // BALL_H
