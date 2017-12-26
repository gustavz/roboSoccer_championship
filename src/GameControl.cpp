#include "lib/GameControl.h"
#include <thread>

GameControl::GameControl(Referee* ref, Physics* physics, eTeam colorT)
    :
      RunnableObject(REFEREE_INIT, 30000),
      penaltySM(PENALTY_STATES::INIT, 30000)
{
    ref_ = ref;
    physics_ = physics;

    ourSide_ = RIGHT_SIDE;

    ourTeam_ = colorT;
    cout << "GameControl constructor ourTeam: " << ourTeam_ << endl;
}

void GameControl::run()
{
    gk_->setSide(ourSide_);
    fp1_->setSide(ourSide_);
    fp2_->setSide(ourSide_);

    while (status_ == RunStatus::RUN)
    {
        if (measuringActive_)
        {
            //Put here what you want to measure
            //std::cout << physics_->getLeftHalfPtr()->isInside(physics_->getBallPositionFiltered().toPosition()) << std::endl;
        }
        restartTimer();
        if (!debugActive_)
        {
            if (updateRef())
            {
                changeState(playmode_);
            }
        }

        SM_DURING
            case GAMECONTROL_STATES::REFEREE_INIT :
                break;
            case GAMECONTROL_STATES::BEFORE_PENALTY :
                if (gk_->isAtTarget())
                {
                    if (ourTeam_ == BLUE_TEAM)
                    {
                        ref_->SetBlueReady();
                    }
                    else
                    {
                        ref_->SetRedReady();
                    }
                }
                break;
            case GAMECONTROL_STATES::PENALTY :
                penalty();
                break;
            case GAMECONTROL_STATES::BEFORE_KICK_OFF :
                if (gk_->isAtTarget() && fp1_->isAtTarget() && fp2_->isAtTarget())
                {
                    if(ourTeam_ == BLUE_TEAM)
                        ref_->SetBlueReady();
                    else
                        ref_->SetRedReady();
                }
                break;
            case GAMECONTROL_STATES::KICK_OFF :
                break;
            case GAMECONTROL_STATES::PLAY_ON :
                break;
            case GAMECONTROL_STATES::DEBUG_CRUISE :
                break;
            case GAMECONTROL_STATES::DEBUG_INTERCEPT:
                break;
            case GAMECONTROL_STATES::ATTACKER_MODE:
            break;
        SM_EXIT
        SM_ENTRY
            case GAMECONTROL_STATES::ATTACKER_MODE:
                cout << "fp2 attacker Mode" << endl;
                fp2_->startAttackerMode();
                break;
            case GAMECONTROL_STATES::DEFENDER_MODE:
                cout << "fp1 defender Mode" << endl;
                fp1_->startDefenderMode(DEFEND_ALONE);
            break;
            case GAMECONTROL_STATES::DEBUG_SHOOT:
                fp2_->startShootBall();
            break;
            case GAMECONTROL_STATES::DEBUG_PASSTO:
                fp1_->startPassTo(fp2_);
            break;
            case GAMECONTROL_STATES::REFEREE_INIT :
                gk_->setSide(ourSide_);
                fp1_->setSide(ourSide_);
                fp2_->setSide(ourSide_);
                cout << "Referee Init  " << endl;
                break;
            case GAMECONTROL_STATES::BEFORE_PENALTY :
                cout << "GAMECONTROL_STATES::BEFORE_PENALTY" << endl;
                beforePenalty();
                break;
            case GAMECONTROL_STATES::PENALTY :
                cout << "GAMECONTROL_STATES::PENALTY" << endl;
                penaltySM.changeState(PENALTY_STATES::INIT);
                break;
            case GAMECONTROL_STATES::BEFORE_KICK_OFF :
                gk_->stopGoalKeeper();
                fp1_->stopAllActions();
                fp2_->stopAllActions();
                beforeKickOff();
                cout << "GAMECONTROL_STATES::BEFORE_KICK_OFF" << endl;
                break;
            case GAMECONTROL_STATES::KICK_OFF :
            cout << "GAMECONTROL_STATES::KICK_OFF" << endl;
                kickOff();
                break;
            case GAMECONTROL_STATES::PLAY_ON :
            cout << "GAMECONTROL_STATES::PLAY_ON" << endl;
                if (autoStrategy_)
                    chooseStrategy();
                playOn();
                break;
            case GAMECONTROL_STATES::DEBUG_CRUISE :
                gk_->stopGoalKeeper();
                gk_->activateCA(true, true, true, true, true, true);
                gk_->setTargetPoint(TargetPoint(Position(1,0.3), 0.15, false));
                gk_->addTargetPoint(TargetPoint(Position(-1.,-0.3), 0.1, false));
                gk_->addTargetPoint(TargetPoint(Position(1,0.3), 0.15, false));
                gk_->addTargetPoint(TargetPoint(Position(-1.,-0.3), 0.1, false));
                gk_->addTargetPoint(TargetPoint(Position(1,0.3), 0.15, false));
                //gk_->activateCA();
                //cout << "debugCruise" << endl;
                break;
            case GAMECONTROL_STATES::DEBUG_INTERCEPT :
                fp1_->setSide(RIGHT_SIDE);
                fp1_->activate();
                fp1_->startClearBall();
                break;

        SM_END
        usleep(getSleepTime());
    }
}

bool GameControl::updateRef()
{
    if (ourTeam_ == BLUE_TEAM)
    {
        ourSide_ = ref_->GetBlueSide();
    }
    else
    {
        ourSide_ = (ref_->GetBlueSide() == LEFT_SIDE) ? (RIGHT_SIDE) : (LEFT_SIDE);
    }
    playmode_ = ref_->GetPlayMode();
    gk_->setSide(ourSide_);
    fp1_->setSide(ourSide_);
    fp2_->setSide(ourSide_);
    return playmode_ != currentState_;
}

void GameControl::setPermanentStrategy(STRATEGIES::Strategies strategy)
{
    strategy_ = strategy;
    autoStrategy_ = false;
}

void GameControl::chooseStrategy()
{
    if (ourSide_ == LEFT_SIDE)
    {
        if (ref_->GetLeftSideGoals() >= ref_->GetRightSideGoals()+4)
        {
            strategy_ = STRATEGIES::DEFENSIVE;
        }
        else
        {
            strategy_ = STRATEGIES::OFFENSIVE;
        }
    }
    else
    {
        if (ref_->GetRightSideGoals() >= ref_->GetLeftSideGoals()+4)
        {
            strategy_ = STRATEGIES::DEFENSIVE;
        }
        else
        {
            strategy_ = STRATEGIES::OFFENSIVE;
        }
    }

}

void GameControl::playOn()
{
    if (strategy_ == STRATEGIES::OFFENSIVE)
    {
        fp1_->startAttackerMode();
        fp2_->startDefenderMode(DEFEND_ALONE);

        fp1_->setDefender(fp2_);
        fp2_->setAttacker(fp1_);
        gk_->setFirstDefender(fp2_);
        gk_->setAttacker(fp1_);
    }
    else
    {
        fp1_->startDefenderMode(DEFEND_FRONT);
        fp2_->startDefenderMode(DEFEND_BACK);

        fp1_->setDefender(fp2_);
        fp2_->setDefender(fp1_);
        gk_->setFirstDefender(fp2_);
        gk_->setSecondDefender(fp1_);
    }
    gk_->startGoalKeeper();
}

void GameControl::beforePenalty()
{
    penaltyTeamShoot_ = ref_->GetSide();
    // Koordinaten der linken Seite.
    Position startPositionKicker(0, 0);
    Position startPositionFp1(1, 0);
    Position startPositionFp2(1, -0.3);
	cout << "penaltyTeamShoot:" << penaltyTeamShoot_ << endl;

    // Fahre beide Fieldplayer aus dem Weg.
    fp1_->setTargetPoint( TargetPoint(startPositionFp1) );
    fp2_->setTargetPoint( TargetPoint(startPositionFp2) );
	penaltySM.changeState(PENALTY_STATES::INIT);
    if (penaltyTeamShoot_ == ourSide_)
    {
        //Wir gehen auf Startposition für schuss und fahren die Roboter aus dem Weg.
        //gk_->setState(GoalKeeper::PENALTY);
        gk_->stopAllActions();
        gk_->activate();
        gk_->setTargetPoint( TargetPoint(startPositionKicker));

    }
    else if (penaltyTeamShoot_ != ourSide_)
    {
        gk_->startGoalKeeper();
        gk_->setSide(LEFT_SIDE);
        gk_->activate();
    }
}

TargetPoint GameControl::prepareShoot(Position ball, Position target, double distToBall = 0.1)
{
    LineSegment seg(ball,target);

    TargetPoint result;
    result.Heading = seg.getDirectionVector();
    result.Location = (seg.getStartVector() - seg.getDirectionVector() * distToBall).toPosition();

    return result;

}

void GameControl::penalty()
{
    Agent* penaltyPlayer = gk_;

    penaltyPlayer->setDesiredSpeed(0.5);

    SUBSM_DURING(penaltySM)
        case PENALTY_STATES::INIT:
			if (penaltyTeamShoot_ == ourSide_)
			{
				penaltySM.changeState(PENALTY_STATES::PREPARE_KICK);
			}
			else
			{
				penaltySM.changeState(PENALTY_STATES::PASSIVE);
			}
            break;
        case PENALTY_STATES::PREPARE_KICK:
            if (penaltyPlayer->isAtTarget())
            {
				penaltySM.changeState(PENALTY_STATES::KICK);
                break;
            }
            break;

        case PENALTY_STATES::KICK:
			penaltyPlayer->shoot();
            penaltySM.changeState(PENALTY_STATES::END);
            break;
        case PENALTY_STATES::END:
            break;

    SUBSM_EXIT(penaltySM)
    SUBSM_ENTRY(penaltySM)
        case PENALTY_STATES::INIT:
            penaltyPlayer->activate();
			cout << "PENALTY_STATES::INIT" << endl;
            break;
        case PENALTY_STATES::PREPARE_KICK:
            penaltyPlayer->setDesiredSpeed(0.3);
			penaltyPlayer->setTargetPoint(prepareShoot(physics_->getBallLastPosition(), physics_->getGoalLeftPtr()->getMiddlePoint(), 0.15));
			cout << "PENALTY_STATES::PREPARE_KICK" << endl;
			break;
		case PENALTY_STATES::KICK:
			cout << "PENALTY_STATES::KICK" << endl;
			break;

    SUBSM_END(penaltySM)

}

void GameControl::beforeKickOff()
{

    Position leftAttack0(-1.2, 0); //goalie
    Position leftAttack1(-0.2, 0); //kickoff
    Position leftAttack2(-0.1, 0.6); //support

    Position leftDefend0(-1.2, 0);
    Position leftDefend1(-0.25, 0.1);
    Position leftDefend2(-0.25, -0.1);

    Position rightAttack0(1.2, 0); //goalie
    Position rightAttack1(0.2, 0); //kickoff
    Position rightAttack2(0.1, 0.6); //support

    Position rightDefend0(1.2, 0);
    Position rightDefend1(0.25, 0.1);
    Position rightDefend2(0.25, -0.1);

    gk_->stopAllActions();
    fp1_->stopAllActions();
    fp2_->stopAllActions();
    gk_->stopGoalKeeper();
    if (ourSide_ == ref_->GetSide())
    {
        if (ourSide_ == LEFT_SIDE)
        {
            gk_->setTargetPoint(leftAttack0);
            fp1_->setTargetPoint(leftAttack1);
            fp2_->setTargetPoint(leftAttack2);
        }
        else
        {
            gk_->setTargetPoint(rightAttack0);
            fp1_->setTargetPoint(rightAttack1);
            fp2_->setTargetPoint(rightAttack2);
        }
    }
    else
    {
        if (ourSide_ == LEFT_SIDE)
        {
            gk_->setTargetPoint(leftDefend0);
            fp1_->setTargetPoint(leftDefend1);
            fp2_->setTargetPoint(leftDefend2);
        }
        else
        {
            gk_->setTargetPoint(rightDefend0);
            fp1_->setTargetPoint(rightDefend1);
            fp2_->setTargetPoint(rightDefend2);
        }
    }
}

void GameControl::kickOff()
{
    eSide kickOffSide = ref_->GetSide();

    if (ourSide_ == kickOffSide)
    {
          fp1_->startKickOff();
    }
}



Position GameControl::getInterceptionParallelTarget(int leftOfBall) const
{
    Vector2d predBallPos(physics_->getPredBallPosition(1000));
    return (predBallPos - physics_->getSimpleBallTrajectory().getNormalVector() * leftOfBall * physics_->ROBOT_OBSTACLE_RADIUS).toPosition();
}



