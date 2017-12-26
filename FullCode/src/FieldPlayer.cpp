#include "lib/FieldPlayer.h"


FieldPlayer::FieldPlayer(RTDBConn& DBC, const int deviceNr, Physics* physics)
    :
      Agent(DBC,deviceNr,physics,0,30002),
      attackerModeSM(ATTACKER_STATES::ANTICIPATE, 30000),
      defenderModeSM(DEFENDER_STATES::SUPPORT_GK, 30000),
      kickOffSM(KICKOFF_STATES::PREPARE, 30000)
{
}

FieldPlayer::~FieldPlayer()
{
}

void FieldPlayer::run()
{
    while (status_ == RunStatus::RUN)
    {
        restartTimer();
        update();

        //Check for activity states
        if (attackerModeActive_)
        {
            attackerMode();
        }
        else if (defenderModeActive_)
        {
            defenderMode();
        }
        else if (kickOffActive_)
        {
            kickOff();
        }

        //cruise if active_
        if (active_)
        {
            cruise();
        }

        usleep(getSleepTime());
    }
}

void FieldPlayer::startSupportGk(double distToHalfLine)
{
    Vector2d addition = ownGoalSegment_->getNormalVector() * distToHalfLine;
    if (ourSide_ == LEFT_SIDE)
    {
        defenseLine_ = LineSegment(physics_->getHalfwayLinePtr()->getEndVector() + addition,
                                   physics_->getHalfwayLinePtr()->getStartVector() + addition);
    }
    else
    {
        defenseLine_ = LineSegment(physics_->getHalfwayLinePtr()->getStartVector() + addition,
                                   physics_->getHalfwayLinePtr()->getEndVector() + addition);
    }
}

bool FieldPlayer::startBlockEnemy()
{
    return false;
}

void FieldPlayer::startAttackerMode()
{
    stopAllActions();
    attackerModeActive_ = true;
}


void FieldPlayer::startDefenderMode(DefenderRole role)
{
    stopAllActions();
    defenderModeActive_ = true;
    defenderRole_ = role;
    defenderModeSM.changeState(DEFENDER_STATES::SUPPORT_GK);
}

void FieldPlayer::attackerMode()
{
    Vector2d aimVec = Vector2d(0.,0.);
    bool antLeftSide = true;
    bool changeSide = true;
    static bool init = true;
    SUBSM_DURING(attackerModeSM)
        case ATTACKER_STATES::ANTICIPATE:
            setDesiredSpeed(0.8);
            if ((enemyFieldHalf_->isInside(physics_->getPredBallPosition(1000))))
            {
                attackerModeSM.changeState(ATTACKER_STATES::SHOOT);
                break;
            }
            if (leftSide_->isInside(physics_->getBall()->GetPos()))
            {
                if (!antLeftSide)
                    changeSide = true;
                antLeftSide = true;
                aimVec = Vector2d(0.,-0.4);
            }
            else
            {
                if (antLeftSide)
                    changeSide = true;
                antLeftSide = false;
                aimVec = Vector2d(0.,0.4);
            }
//            if ((isAtTarget()) && (enemyFieldHalf_->isInside(physics_->getBallLastPosition())))
//            {
//                attackerModeSM.changeState(ATTACKER_STATES::SHOOT_INIT);
//            }
            if (changeSide || init)
            {
                activateCA(true, true, true, true, true, true);
                setTargetPoint(TargetPoint(aimVec, (enemyGoalSegment_->getMiddleVector() - position_).getNormalized()));
                changeSide = false;
                init = false;
            }
        break;
        case ATTACKER_STATES::SHOOT:
            if (ownFieldHalf_->isInside(physics_->getPredBallPosition(1000)))
            {
                attackerModeSM.changeState(ATTACKER_STATES::ANTICIPATE);
                break;
            }
            shootBall();
        break;
    SUBSM_EXIT(attackerModeSM)
    SUBSM_ENTRY(attackerModeSM)
    SUBSM_END(attackerModeSM)

}

void FieldPlayer::defenderMode()
{
    SUBSM_DURING(defenderModeSM)
        case DEFENDER_STATES::SUPPORT_GK:

//            if (!defenseLine_.isLeftOfLine(physics_->getBallPositionFiltered()))
//            {
//                //Ball is behind defense line
//                if (defenderRole_ == DEFEND_ALONE)
//                {
//					defenderModeSM.changeState(DEFENDER_STATES::PASS_BALL);
//                    break;
//                }
//                else if (defenderRole_ == DEFEND_FRONT)
//                {
//                    //BLOCK ENEMY if allowed
//                    //changeState(DEFENDER_STATES::BLOCK_ENEMY);
//					defenderModeSM.changeState(DEFENDER_STATES::SHOOT_ON_GOAL);
//                    break;
//                }
//                else //DEFEND_BACK
//                {
//					defenderModeSM.changeState(DEFENDER_STATES::SHOOT_ON_GOAL);
//                    break;
//                }
//            }
//			else if(defenderRole_ == DEFEND_ALONE &&
//					ownFieldHalf_->isInside(physics_->getBallPositionFiltered().toPosition()) &&
//					fabs(physics_->getBallVelocity()*physics_->getHalfwayLinePtr()->getNormalVector()) < 0.2 &&
//                                        physics_->getClosestEnemysDistanceToBall() > 0)
//			{
//				defenderModeSM.changeState(DEFENDER_STATES::SHOOT_ON_GOAL);
//				break;
//			}
//            supportGk();


        //-------------TEST GUSTAV-----------------//
            if (!defenseLine_.isLeftOfLine(physics_->getBallPositionFiltered().toPosition())){
                defenderModeSM.changeState(DEFENDER_STATES::SHOOT_ON_GOAL);
                break;
            }
            else if(defenderRole_ == DEFEND_ALONE &&
                            ownFieldHalf_->isInside(physics_->getBallPositionFiltered().toPosition()) &&
                            fabs(physics_->getBallVelocity()*physics_->getHalfwayLinePtr()->getNormalVector()) < 0.2)
            {
                    defenderModeSM.changeState(DEFENDER_STATES::SHOOT_ON_GOAL);
                    break;
            }
            supportGk();
        //-------------ENDE TEST GUSTAV-----------------//

        break;
        case DEFENDER_STATES::CLEAR_BALL:
            if (defenseLine_.isLeftOfLine(physics_->getBallPositionFiltered()))
            {
				defenderModeSM.changeState(DEFENDER_STATES::SUPPORT_GK);
                break;
            }

            //clearBall();
            shootBall();
        break;
        case DEFENDER_STATES::PASS_BALL:
            if (physics_->getClosestEnemysDistanceToBall() < 0.3)
            {
				defenderModeSM.changeState(DEFENDER_STATES::CLEAR_BALL);
                break;
            }
            if (physics_->getHalfwayLinePtr()->isLeftOfLine(physics_->getPredBallPosition(1000)))
            {
				defenderModeSM.changeState(DEFENDER_STATES::SUPPORT_GK);
                break;
            }


            shootBall();
            //passTo();
            //clearBall();
        break;
        case DEFENDER_STATES::SHOOT_ON_GOAL:

//            if (physics_->getClosestEnemysDistanceToBall() < 0.3)
//            {
//				defenderModeSM.changeState(DEFENDER_STATES::CLEAR_BALL);
//                break;
//            }
//            if (physics_->getHalfwayLinePtr()->isLeftOfLine(physics_->getPredBallPosition(1000)))
//            {
//				defenderModeSM.changeState(DEFENDER_STATES::SUPPORT_GK);
//                break;
//            }
//            shootBall();

                //-------------TEST GUSTAV-----------------//
            if (enemyFieldHalf_->isInside(physics_->getBallPositionFiltered().toPosition()))
            {
                                defenderModeSM.changeState(DEFENDER_STATES::SUPPORT_GK);
                break;
            }
            shootBall();
            //-------------ENDE TEST GUSTAV-----------------//

        break;
    SUBSM_EXIT(defenderModeSM)
    SUBSM_ENTRY(defenderModeSM)
        case DEFENDER_STATES::SUPPORT_GK:
            cout << "defender supports GK" << endl;
            if (defenderRole_ == DEFEND_ALONE)
            {
                startSupportGk(0.4);
            }
            else if (defenderRole_ == DEFEND_FRONT)
            {
                startSupportGk(0.);
            }
            else
            {
                startSupportGk(0.8);
            }
        break;
        case DEFENDER_STATES::CLEAR_BALL:
            cout << "defender clears Ball" << endl;
        break;
        case DEFENDER_STATES::PASS_BALL:
            //startPassTo(attacker_);
            cout << "defender passes Ball" << endl;
        break;
        case DEFENDER_STATES::SHOOT_ON_GOAL:
            cout << "defender shoots Ball" << endl;
        break;
    SUBSM_END(defenderModeSM)
}


void FieldPlayer::anticipatePass()
{

}

void FieldPlayer::prepareShoot()
{

}

void FieldPlayer::setSide(eSide s)
{
    ourSide_ = s;
    leftSide_ = physics_->getLowerHalfPtr();
    rightSide_ = physics_->getUpperHalfPtr();

    if (s == LEFT_SIDE)
    {
        ownGoalSegment_ = physics_->getGoalLeftPtr();
        enemyGoalSegment_ = physics_->getGoalRightPtr();
        ownPenaltyZone_ = physics_->getObstaclePenaltyAreaLeftPtr();
        enemyPenaltyZone_ = physics_->getObstaclePenaltyAreaRightPtr();
        ownFieldHalf_ = physics_->getLeftHalfPtr();
        enemyFieldHalf_ = physics_->getRightHalfPtr();
    }
    else
    {
        ownGoalSegment_ = physics_->getGoalRightPtr();
        enemyGoalSegment_ = physics_->getGoalLeftPtr();
        ownPenaltyZone_ = physics_->getObstaclePenaltyAreaRightPtr();
        enemyPenaltyZone_ = physics_->getObstaclePenaltyAreaLeftPtr();
        ownFieldHalf_ = physics_->getRightHalfPtr();
        enemyFieldHalf_ = physics_->getLeftHalfPtr();
    }
}

void FieldPlayer::supportGk()
{
    boost::optional<Vector2d> intersection = boost::none;
    for (auto &k : physics_->getBallTrajectory(5000))
    {
        if (k.intersects(defenseLine_))
        {
            intersection = k.getIntersection(defenseLine_);
            break;
        }
    }

    if (!intersection)
    {
        intersection = defenseLine_.getIntersection(LineSegment(physics_->getBall()->GetPos(),ownGoalSegment_->getMiddlePoint()));
        if (!intersection)
        {
            setTargetPoint(TargetPoint(defenseLine_.getClosestPoint(physics_->getBall()->GetPos())));
        }
        else
        {
            setTargetPoint(TargetPoint(*intersection));
        }
        desiredSpeed_ = 1.0;

        return;
    }
    TargetPoint target(*intersection,0.03);
    setTargetPoint(target);
    desiredSpeed_ = 1.5;
}

void FieldPlayer::blockEnemy()
{

}

void FieldPlayer::startKickOff()
{
    std::cout << "Start Kickoff!" << std::endl;
    stopAllActions();
    kickOffSM.changeState(KICKOFF_STATES::PREPARE);
    kickOffActive_ = true;
}

void FieldPlayer::kickOff()
{
    SUBSM_DURING(kickOffSM)
        case KICKOFF_STATES::PREPARE :
            if (isAtTarget())
            {
                kickOffSM.changeState(KICKOFF_STATES::SHOOT);
                break;
            }
        break;
        case KICKOFF_STATES::SHOOT :
        break;
    SUBSM_EXIT(kickOffSM)
    SUBSM_ENTRY(kickOffSM)
        case KICKOFF_STATES::PREPARE :
            activateCA(true, true, true, true, true, true);
            calcKickOffPreparePosition();
        break;
        case KICKOFF_STATES::SHOOT :
            shoot();
        break;

    SUBSM_END(kickOffSM)
}

void FieldPlayer::calcKickOffPreparePosition()
{
    LineSegment lineToGoal(physics_->getBallPositionFiltered(), enemyGoalSegment_->getMiddleVector());

    TargetPoint kickOffPrepareTarget;

    Vector2d EnemiesCenter;

    for (auto k : physics_->getEnemies())
    {
        EnemiesCenter = EnemiesCenter + Vector2d(k->GetPos());
    }
    EnemiesCenter = EnemiesCenter / physics_->getNumberOfEnemies();

    Line parallelLine(lineToGoal.getMiddleVector(), enemyGoalSegment_->getDirectionVector().getAngle());
    std::vector<Vector2d> intersections = physics_->getFieldPtr()->getIntersection(parallelLine);

    if (intersections.empty())
    {
        //Shoot over middle
        kickOffPrepareTarget.Location = lineToGoal.getStartVector() - (lineToGoal.getDirectionVector() * 0.2);
        kickOffPrepareTarget.Heading = lineToGoal.getDirectionVector();
    }
    else if (intersections.size() == 1)
    {
        //Only one option
        LineSegment lineToWall(lineToGoal.getStartVector(), intersections.front());
        kickOffPrepareTarget.Location = lineToWall.getStartVector() - (lineToWall.getDirectionVector() * 0.2);
        kickOffPrepareTarget.Heading = lineToWall.getDirectionVector();
    }
    else
    {
        LineSegment lineToWall;
        if (lineToGoal.isLeftOfLine(intersections.front()) != lineToGoal.isLeftOfLine(EnemiesCenter))
        {
            lineToWall = LineSegment(lineToGoal.getStartVector(), intersections.front());
        }
        else
        {
            lineToWall = LineSegment(lineToGoal.getStartVector(), intersections.back());
        }
        kickOffPrepareTarget.Location = lineToWall.getStartVector() - (lineToWall.getDirectionVector() * 0.2);
        kickOffPrepareTarget.Heading = lineToWall.getDirectionVector();
    }

    kickOffPrepareTarget.Precision = 0.01;
    kickOffPrepareTarget.AnglePrecision = 0.05;

    setTargetPoint(kickOffPrepareTarget);

}






