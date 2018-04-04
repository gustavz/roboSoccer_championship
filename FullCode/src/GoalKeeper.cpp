#include "lib/GoalKeeper.h"

GoalKeeper::GoalKeeper(RTDBConn& DBC, const int deviceNr, Physics* physics)
    :
      Agent(DBC,deviceNr, physics, GOALKEEPER_STATES::AUTO_HOLD_NOT_ACTIVE, 30001),
      physics_(physics),
      goalKeeperActive_(false),
      gkKickSM(GOALKEEPER_KICK_STATES::PREPARE, 30000)
{
}

GoalKeeper::~GoalKeeper()
{
}

void GoalKeeper::setSide(eSide s)
{
    const double distToGoalLine = 0.04;
    const double distToPost = 0.15;

    ourSide_ = s;
	leftSide_ = physics_->getLowerHalfPtr();
	rightSide_ = physics_->getUpperHalfPtr();

    if (s == LEFT_SIDE)
    {
        ownInnerGoal_ = physics_->getInnerGoalLeftPtr();
        ownGoalSegment_ = physics_->getGoalLeftPtr();
        enemyGoalSegment_ = physics_->getGoalRightPtr();
        ownPenaltyZone_ = physics_->getObstaclePenaltyAreaLeftPtr();
        enemyPenaltyZone_ = physics_->getObstaclePenaltyAreaRightPtr();
		ownFieldHalf_ = physics_->getLeftHalfPtr();
		enemyFieldHalf_ = physics_->getRightHalfPtr();
    }
    else
    {
        ownInnerGoal_ = physics_->getInnerGoalRightPtr();
        ownGoalSegment_ = physics_->getGoalRightPtr();
        enemyGoalSegment_ = physics_->getGoalLeftPtr();
        ownPenaltyZone_ = physics_->getObstaclePenaltyAreaRightPtr();
        enemyPenaltyZone_ = physics_->getObstaclePenaltyAreaLeftPtr();
		ownFieldHalf_ = physics_->getRightHalfPtr();
		enemyFieldHalf_ = physics_->getLeftHalfPtr();
    }

    Vector2d leftEdge = ownGoalSegment_->getSupportVector()
                        - (ownGoalSegment_->getNormalVector() * distToGoalLine)
                        - (ownGoalSegment_->getDirectionVector() * distToPost);
    Vector2d rightEdge = ownGoalSegment_->getEndVector()
                        - (ownGoalSegment_->getNormalVector() * distToGoalLine)
                        + (ownGoalSegment_->getDirectionVector() * distToPost);
    protectionSegment_ = LineSegment(leftEdge, rightEdge);
}

void GoalKeeper::run()
{
    while (status_ == RunStatus::RUN)
    {
        restartTimer();
        update();
        SM_DURING
            case GOALKEEPER_STATES::AUTO_HOLD_ACTIVE:
                if (!goalKeeperActive_)
                {
                    changeState(GOALKEEPER_STATES::AUTO_HOLD_NOT_ACTIVE);
                    break;
                }
                if (shouldClearBall())
                {
                    changeState(GOALKEEPER_STATES::CLEAR_BALL);
                    break;
                }

                if (trajectoryOnGoal())
                {
                    anticipateBall();
                }
                else
                {
                    followBall();
                }
                break;
            case GOALKEEPER_STATES::AUTO_HOLD_NOT_ACTIVE:
                if (goalKeeperActive_)
                {
                    changeState(GOALKEEPER_STATES::AUTO_HOLD_ACTIVE);
                    break;
                }
                break;
            case GOALKEEPER_STATES::CLEAR_BALL:
                if (!shouldClearBall())
                {
                    changeState(GOALKEEPER_STATES::AUTO_HOLD_ACTIVE);
                    std::cout << "Stop clear ball" << std::endl;
                    break;
                }
                if (!goalKeeperActive_)
                {
                    changeState(GOALKEEPER_STATES::AUTO_HOLD_NOT_ACTIVE);
                    break;
                }
                gkKick();
                break;
			case GOALKEEPER_STATES::PENALTY :

				if(physics_->getBallVelocity().getLength() > 0.1)
				{
					deleteTargetPoints();
					changeState(GOALKEEPER_STATES::AUTO_HOLD_NOT_ACTIVE);
					std::cout << "PENALTY DONE" << std::endl;
					break;
				}
				shootBall();
				break;
        SM_EXIT
        SM_ENTRY
            case GOALKEEPER_STATES::AUTO_HOLD_NOT_ACTIVE:
                activateCA(true,true,false,true,false,true);
                break;
            case GOALKEEPER_STATES::AUTO_HOLD_ACTIVE:
                activateCA(false,false,false,true,false,true);

                std::cout << "AutoHold active" << std::endl;
                followBall();
                break;
            case GOALKEEPER_STATES::CLEAR_BALL:
                activateCA(false,false,false,true,true,false);
                std::cout << "Goaly clear ball started" << std::endl;
                gkKickSM.changeState(GOALKEEPER_KICK_STATES::PREPARE);
                break;
			case GOALKEEPER_STATES::PENALTY :
					shootBallSM.changeState(SHOOTBALL_STATES::INIT);
					std::cout << "GK Start Penalty" << std::endl;
				break;
        SM_END

        if (active_)
        {
            cruise();
        }

        usleep(getSleepTime());
    }
}

void GoalKeeper::startPenaltyMode(){
	stopAllActions();
	stopGoalKeeper();
	penaltyModeActive_ = true;
	changeState(GOALKEEPER_STATES::PENALTY);
}

void GoalKeeper::startGoalKeeper()
{
    goalKeeperActive_ = true;
}

void GoalKeeper::stopGoalKeeper()
{
    goalKeeperActive_ = false;
}

void GoalKeeper::followBall()
{
    boost::optional<Vector2d> intersection = protectionSegment_.getIntersection(LineSegment(physics_->getBall()->GetPos(),ownGoalSegment_->getClosestPoint(physics_->getBallPositionFiltered())));
    TargetPoint tp;
    if (!intersection)
    {
        tp = TargetPoint(protectionSegment_.getClosestPoint(physics_->getBallPositionFiltered()));
    }
    else
    {
        tp = TargetPoint(*intersection);
    }

    tp.Precision = 0.02;
    setTargetPoint(tp);

    desiredSpeed_ = 1;
}

void GoalKeeper::anticipateBall()
{
    boost::optional<Vector2d> intersection = boost::none;
    for (auto &k : physics_->getBallTrajectory(anticipationTime_))
    {
        if (k.intersects(protectionSegment_))
        {
            intersection = k.getIntersection(protectionSegment_);
            break;
        }
    }

    if (!intersection)
    {
        followBall();
        return;
    }

    TargetPoint tp(*intersection);
    tp.Precision = 0.02;
    setTargetPoint(tp);

    desiredSpeed_ = 1.5;
}

bool GoalKeeper::trajectoryOnGoal()
{
    for (auto &k : physics_->getBallTrajectory(anticipationTime_))
    {
        if (k.intersects(*ownGoalSegment_))
        {
            return true;
        }
    }
    return false;
}

bool GoalKeeper::shouldClearBall()
{
    if (!ownPenaltyZone_)
    {
        return false;
    }
    return ownPenaltyZone_->isInside(physics_->getBallPositionFiltered().toPosition()) && (physics_->getBallVelocity().getLength() < 0.2) ;
}

void GoalKeeper::gkKick()
{
    SUBSM_DURING(gkKickSM)
        case GOALKEEPER_KICK_STATES::PREPARE:
            if (physics_->getBallPositionFiltered().getDistance(lastBallPosBeforeKick_) > 0.05)
            {
                gkKickSM.changeState(GOALKEEPER_KICK_STATES::PREPARE);
                break;
            }
            if (isAtTarget())
            {
                gkKickSM.changeState(GOALKEEPER_KICK_STATES::SHOOT);
                break;
            }
        break;
        case GOALKEEPER_KICK_STATES::SHOOT:
            if (after(3000000))
            {
                gkKickSM.changeState(GOALKEEPER_KICK_STATES::PREPARE);
                break;
            }
        break;
    SUBSM_EXIT(gkKickSM)
    SUBSM_ENTRY(gkKickSM)
        case GOALKEEPER_KICK_STATES::PREPARE:
            activateCA(true, true, false, true, true, false);
            calcPrepareGkKick();
        break;
        case GOALKEEPER_KICK_STATES::SHOOT:
            shoot();
        break;
    SUBSM_END(gkKickSM)
}

void GoalKeeper::calcPrepareGkKick()
{
    lastBallPosBeforeKick_ = physics_->getBallPositionFiltered();
    LineSegment goalSeg(lastBallPosBeforeKick_, enemyGoalSegment_->getMiddleVector());

    Vector2d kickPosition = goalSeg.getStartVector() - goalSeg.getDirectionVector() * 0.1;
    Vector2d kickDirection = goalSeg.getDirectionVector();

    Vector2d closestPoint = ownInnerGoal_->getClosestPoint(lastBallPosBeforeKick_);
    if (!ownInnerGoal_->isInSegmentArea(lastBallPosBeforeKick_))
    {
        kickPosition = protectionSegment_.getMiddleVector();
    }
    else if (ownInnerGoal_->getDistance(lastBallPosBeforeKick_) < 0.1)
    {
        if (closestPoint.getDistance(ownInnerGoal_->getStartVector()) < ownInnerGoal_->getLength() / 2)
        {
            kickPosition = closestPoint + ownInnerGoal_->getDirectionVector() * 0.1;
        }
        else
        {
            kickPosition = closestPoint - ownInnerGoal_->getDirectionVector() * 0.1;
        }
    }
    else
    {
        kickPosition = closestPoint;
    }

    kickDirection = (lastBallPosBeforeKick_ - kickPosition).getNormalized();

    double dist = kickPosition.getDistance(lastBallPosBeforeKick_);
    if (dist > 0.1)
    {
        kickPosition = kickPosition + (kickDirection * (dist - 0.1));
    }

    TargetPoint target(kickPosition, kickDirection, 0.04);
    target.AnglePrecision = 0.05;
    setTargetPoint(target);
}


