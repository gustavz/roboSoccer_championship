#include "lib/Debug.h"
#include <iostream>

Debug::Debug(GameControl* gameControl)
    :
      RunnableObject(DEBUG_STATES::NO_DEBUG, 30000)
{
    gameControl_ = gameControl;
}

void Debug::run()
{
    int predictionMilliseconds = 1000;
    while (status_ == RunStatus::RUN)
    {
        std::string message;
        std::string value;
        std::cin >> message;
        if (message == "x")
        {
            gameControl_->gk_->AbortGotoXY();
            gameControl_->fp1_->AbortGotoXY();
            gameControl_->fp2_->AbortGotoXY();
            changeState(DEBUG_STATES::NO_DEBUG);
            cout << "stopped" << endl;
        }
        else if (message == "at")
        {
            gameControl_->debugActive_ = true;
            gameControl_->changeState(GAMECONTROL_STATES::ATTACKER_MODE);
        }
        else if (message == "df")
        {
            gameControl_->debugActive_ = true;
            gameControl_->changeState(GAMECONTROL_STATES::DEFENDER_MODE);
        }
        else if (message == "c")
        {
            gameControl_->gk_->activate();
            changeState(DEBUG_STATES::CRUISE);
        }
        else if (message == "s")
        {
            gameControl_->debugActive_ = true;
            gameControl_->changeState(GAMECONTROL_STATES::DEBUG_SHOOT);
        }
        else if (message == "pt")
        {
            gameControl_->debugActive_ = true;
            gameControl_->changeState(GAMECONTROL_STATES::DEBUG_PASSTO);
        }
        else if (message == "ref")
        {
            gameControl_->debugActive_ = true;
            changeState(DEBUG_STATES::REFEREE);
        }
        else if (message == "p")
        {
            changeState(DEBUG_STATES::PENALTY);
        }
        else  if (message == "away")
        {
            gameControl_->gk_->GotoXY(((double) rand() / (RAND_MAX)) - 1,((double) rand() / (RAND_MAX)) - 1);
            gameControl_->fp1_->GotoXY(((double) rand() / (RAND_MAX)),((double) rand() / (RAND_MAX)) - 1);
            gameControl_->fp2_->GotoXY(((double) rand() / (RAND_MAX)) - 1,((double) rand() / (RAND_MAX)));
        }
        else  if (message == "start")
        {

                changeState(DEBUG_STATES::START);
        }
        else  if (message == "i")
        {
            gameControl_->debugActive_ = true;
            changeState(DEBUG_STATES::INTERCEPT);
        }
        else  if (message == "m")
        {
            gameControl_->measuringActive_ = true;
        }
        else if (message == "bp")
        {
            gameControl_->debugActive_ = true;
            gameControl_->fp1_->deactivate();
            gameControl_->fp2_->deactivate();
            gameControl_->gk_->deactivate();
            changeState(DEBUG_STATES::PREDICTION);
        }

        SM_DURING
            case DEBUG_STATES::CRUISE :
                gameControl_->debugCruiseSpeed_ =  (double)stoi(message)/100.;
                gameControl_->gk_->desiredSpeed_ = (double)stoi(message)/100.;
                std::cout << gameControl_->debugCruiseSpeed_ << std::endl;
                break;
            case DEBUG_STATES::PREDICTION :
                predictionMilliseconds = stoi(message);

                std::cout << std::endl << std::endl << "<<<<<<<<<<<<<Prediction Started>>>>>>>>>>>>>" << std::endl;

                gameControl_->physics_->startComparePrediction(predictionMilliseconds);
                break;
            case DEBUG_STATES::REFEREE :
                updateRef(message);
                break;
        SM_EXIT
        SM_ENTRY
            case DEBUG_STATES::NO_DEBUG :
                gameControl_->debugActive_ = false;
                break;
            case DEBUG_STATES::CRUISE :
                gameControl_->debugActive_ = true;
                gameControl_->changeState(GAMECONTROL_STATES::DEBUG_CRUISE);
                break;
            case DEBUG_STATES::INTERCEPT :
                gameControl_->debugActive_ = true;
                gameControl_->changeState(GAMECONTROL_STATES::DEBUG_INTERCEPT);
                break;
            case DEBUG_STATES::PENALTY :
                gameControl_->debugActive_ = true;
                gameControl_->changeState(GAMECONTROL_STATES::PENALTY);
            break;
            case DEBUG_STATES::START :
                gameControl_->debugActive_ = true;
                gameControl_->changeState(GAMECONTROL_STATES::PLAY_ON);
                break;
            case DEBUG_STATES::PREDICTION :
                std::cout << "Enter prediction Time to start new prediction" << std::endl;
                break;
            case DEBUG_STATES::REFEREE :
                std::cout << "Enter [left, right] to set Side or [init, bko, ko, play, bpen, pen] for referee states" << std::endl;
                break;

        SM_END

    }
}

void Debug::updateRef(std::string message)
{
    if (message == "left")
    {
        gameControl_->ourSide_ = LEFT_SIDE;
		gameControl_->penaltyTeamShoot_ = LEFT_SIDE;
        gameControl_->gk_->setSide(LEFT_SIDE);
        gameControl_->fp1_->setSide(LEFT_SIDE);
        gameControl_->fp2_->setSide(LEFT_SIDE);
    }
    else if (message == "right")
    {
        gameControl_->ourSide_ = RIGHT_SIDE;
		gameControl_->penaltyTeamShoot_ = RIGHT_SIDE;
        gameControl_->gk_->setSide(RIGHT_SIDE);
        gameControl_->fp1_->setSide(RIGHT_SIDE);
        gameControl_->fp2_->setSide(RIGHT_SIDE);
    }
    else if (message == "init")
    {
        gameControl_->changeState(REFEREE_INIT);
    }
    else if (message == "bko")
    {
        gameControl_->changeState(BEFORE_KICK_OFF);
    }
    else if (message == "ko")
    {
        gameControl_->changeState(KICK_OFF);
    }
    else if (message == "play")
    {
        gameControl_->changeState(PLAY_ON);
    }
    else if (message == "bpen")
    {
        gameControl_->changeState(BEFORE_PENALTY);
    }
    else if (message == "pen")
    {
        gameControl_->changeState(PENALTY);
    }

}

