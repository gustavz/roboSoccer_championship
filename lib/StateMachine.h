#ifndef STATEMACHINE_H
#define STATEMACHINE_H


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

    StateMachine(int initState, int interval)
        :
          currentState_(initState),
          lastState_(initState),
          stateChangeFlag_(true),
          interval_(interval)
    {}

    int getState(){return currentState_;}

    void changeState(int newState){currentState_ = newState; stateChangeFlag_ = true; timer_ = 0;}

    //RISKY!!! But Usefull
    void changeToNextState(){currentState_++; stateChangeFlag_ = true; timer_ = 0;}

    bool after(int time_in_micros){return (time_in_micros <= timer_);}

    void incTimer(){timer_ += interval_;}

    int currentState_;

    int lastState_;

    bool stateChangeFlag_;

protected:

    int interval_; //Microseconds

    int timer_;
};

#endif // STATEMACHINE_H
