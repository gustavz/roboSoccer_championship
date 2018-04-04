#ifndef BALL_H
#define BALL_H

#include "raw_ball.h"

class Ball : public RawBall
{

public:

	/**
	  *@brief Constructor for Ball
	  */
    Ball(RTDBConn &DBC) : RawBall(DBC) {}

	/**
	  *@brief  return the timestamp of last ball position update
	  */
    Timestamp getLastTimestamp()
    {
        return mBall.timestamp;
    }


};

#endif // BALL_H
