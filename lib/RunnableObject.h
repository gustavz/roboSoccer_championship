#ifndef RUNNABLEOBJECT_H
#define RUNNABLEOBJECT_H

#include <time.h>
#include "StateMachine.h"
#include <iostream>
#include <math.h>
#include <QTime>

class RunnableObject : public StateMachine
{
public:

    enum RunStatus{
        RUN = 0,
        STOP = 1
    };

    RunnableObject(int initState, int interval)
        :
          StateMachine(initState, interval),
          status_(RunStatus::RUN),
          lastSchedule_(time(0))
    {
        timer_.start();
    }

    virtual void run() = 0;

    void stop(){status_ = STOP;}

    int getSleepTime()
    {

        int ellapsedMicroSecs = timer_.elapsed() * 10e3;
        if (ellapsedMicroSecs > interval_)
        {
            if (noWarning_)
            {
                noWarning_ = false;
                return 0;
            }
            std::cout << "Schedule missed its deadline of " << interval_ << " milliseconds!" << std::endl;
            return 0;
        }
        return interval_ - ellapsedMicroSecs;
    }

    void restartTimer()
    {
        timer_.restart();
    }


protected:

    QTime timer_;

    RunStatus status_;

    time_t lastSchedule_;

    bool noWarning_ = false;
};

#endif // RUNNABLEOBJECT_H
