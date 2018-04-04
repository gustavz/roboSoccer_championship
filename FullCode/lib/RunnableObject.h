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
	/**
	 * @brief Constructor for the runnable object
	 */
    RunnableObject(int initState, int interval)
        :
          StateMachine(initState, interval),
          status_(RunStatus::RUN),
          lastSchedule_(time(0))
    {
        timer_.start();
    }

	/**
	 * @brief run
	 */
    virtual void run() = 0;

	/**
	 * @brief stop run
	 */
    void stop(){status_ = STOP;}

	/**
	 * @brief calculate sleep time depending on calculation time
	 */
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

	/**
	 * @brief restart timer_
	 */
    void restartTimer()
    {
        timer_.restart();
    }


protected:

	QTime timer_; /**< timer to time*/

	RunStatus status_; /**< status of the runnable object*/

	time_t lastSchedule_; /**< time of last schedule*/

	bool noWarning_ = false; /**< remove warning for one execution of run()*/
};

#endif // RUNNABLEOBJECT_H
