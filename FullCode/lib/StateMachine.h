#ifndef STATEMACHINE_H
#define STATEMACHINE_H


/**
  *@brief Macros to help with the usage of StateMachines
  */
#define SM_DURING   if (!stateChangeFlag_)\
                    {\
                    incTimer();\
                    switch (currentState_)\
                    {
#define SM_EXIT     default:\
                    break;\
                    }\
                    }\
                    if (stateChangeFlag_)\
                    {\
                    switch (lastState_)\
                    {
#define SM_ENTRY    default:\
                    break;\
                    }\
                    switch (currentState_)\
                    {
#define SM_END      default:\
                    break;\
                    }\
                    stateChangeFlag_ = false;\
                    }\
                    lastState_ = currentState_;


/**
  *@brief Macros to help with the usage of subStatemachines
  */
#define SUBSM_DURING(a) if (!a.stateChangeFlag_)\
                        {\
                        a.incTimer();\
                        switch (a.currentState_)\
                        {
#define SUBSM_EXIT(a)   default:\
                        break;\
                        }\
                        }\
                        if (a.stateChangeFlag_)\
                        {\
                        switch (a.lastState_)\
                        {
#define SUBSM_ENTRY(a)  default:\
                        break;\
                        }\
                        switch (a.currentState_)\
                        {
#define SUBSM_END(a)    default:\
                        break;\
                        }\
                        a.stateChangeFlag_ = false;\
                        }\
                        a.lastState_ = a.currentState_;

class StateMachine
{

public:

	/**
	  *@brief Constructor for StateMachine
	  */
	StateMachine(int initState, int interval)
        :
          currentState_(initState),
          lastState_(initState),
          stateChangeFlag_(true),
          interval_(interval)
    {}

	/**
	  *@brief get the current state of the StateMachine
	  */
	int getState(){return currentState_;}

	/**
	  *@brief change the state of the StateMachine setting the state change flag and resetting the timer
	  */
	void changeState(int newState){currentState_ = newState; stateChangeFlag_ = true; timer_ = 0;}

	/**
	  *@brief returns true if there was no state change in time_in_micros
	  */
	bool after(int time_in_micros){return (time_in_micros <= timer_);}

	/**
	  *@brief increment timer
	  */
	void incTimer(){timer_ += interval_;}

	int currentState_; /**< current state of the StateMachine*/

	int lastState_; /**< last state of the StateMachine*/

	bool stateChangeFlag_; /**< State change flag being set to true for one period*/

protected:

	int interval_; /**< Statemachine interval */

	int timer_; /**< timer counting intervals*/
};

#endif // STATEMACHINE_H
