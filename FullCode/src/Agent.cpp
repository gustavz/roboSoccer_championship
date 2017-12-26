#include "lib/Agent.h"
#include "lib/Path.h"
#include "lib/Physics.h"

#include "helper/Vector2d.h"

/* DEFINE FOR SIMULATION - otherwise cruise does not work. */
//#define SIMULATION


Agent::Agent(RTDBConn& DBC, const int deviceNr, Physics* physics, int initState, int interval)
    :
      RoboControl(DBC, deviceNr),
      Path(physics, deviceNr),
      RunnableObject(initState,interval),
      desiredSpeed_(0.5),
      active_(true),
      stopBallSM(STOPBALL_STATES::INIT, 30000),
      clearBallSM(CLEARBALL_STATES::INIT, 30000),
      shootBallSM(SHOOTBALL_STATES::INIT,30000),
      passToSM(PASSTO_STATES::INIT, 30000)

{
   timerAgent_.start();
   timerUpdate_.start();
}

Agent::~Agent()
{
}

void Agent::update()
{
    eTeam team = id_ >= 3 ? RED_TEAM : BLUE_TEAM;
    mCam.getPositions(mOwnTeamInfo, team);
    Timestamp actualTimestamp = mOwnTeamInfo.timestamp;
    int64_t timeDiff = lastPositionTimestamp_.diffns(actualTimestamp);

    if (timeDiff <= 0)
    {
        return;
    }

    Vector2d position(GetPos());
    Vector2d heading(GetPhi().Rad());

    movingAverage(position_, position, 0.9);
    movingAverage(heading_, heading, 0.9);
    movingAverage(robotSpeed_, (position_.getDistance(lastPosition_) / (timeDiff/1e9)), 0.5);

    lastPosition_ = position_;

    lastPositionTimestamp_ = actualTimestamp;
}


bool Agent::isAtTarget() const
{
    return targetPoints_.size() == 0;
}


void Agent::cruise()
{

    // Variables for Speed PID Controller
    int motorControllerSpeed = 0;  // -255 - 255

    // If function is not called for more than 100ms
    if (timerAgent_.elapsed() > 100)
    {
        speedIntegrated_ = 0;
    }

    // Timer for controller
    double delta_t = (double)(timerAgent_.elapsed())/1000.;
    timerAgent_.restart();

    if (targetPoints_.size() == 0)
    {
        return;
    }

    const double NO_DIRVE_ANGLE = deg2rad(25.);
    const double BRAKE_DISTANCE = desiredSpeed_ / 3.5;

    TargetPoint currentTarget = targetPoints_.front();

    // compute path with collision avoidance if activated
    if (useCA_)
    {
        bool brake = true;
        compute(currentTarget);

        currentTarget = CAtargetPoints_.back();
        for (auto &k : CAtargetPoints_)
        {
            if (k.Location.getDistance(position_) >= 0.1)
            {
                currentTarget = k;
                brake = false;
                break;
            }
        }

        currentTarget.Brake = brake;
        currentTarget.Precision = 0.03;
    }

    LineSegment goalSeg(position_, currentTarget.Location);
    LineSegment finalGoalSeg(position_, targetPoints_.front().Location);
    //cout << "Curren target: " << currentTarget.Location << endl;
    //cout << "position_: " << position_ << endl;

    double distToFinalGoal = finalGoalSeg.getLength();
    double distance = goalSeg.getLength();
    double angleDiff = heading_.getAngle(goalSeg.getDirectionVector());
    driveBackwards = false;
    if (fabs(angleDiff) > M_PI/2)
    {
        driveBackwards = true;
        angleDiff = angleDiff - sign(angleDiff) * M_PI ;
    }

    //PID Controller for speed
    double speedDifference = desiredSpeed_ - robotSpeed_;
    double speedDerived = (speedDifference - lastSpeed_) / delta_t;
    speedIntegrated_ += (speedDifference * delta_t);
    constraint(speedIntegrated_, -10., 10.);
    motorControllerSpeed = round(desiredSpeed_ * 110 + 50 * speedDifference - 0 * speedDerived + 0 * speedIntegrated_);
    constraint(motorControllerSpeed, -255, 255);
    if (desiredSpeed_ < 0.03)
    {
        motorControllerSpeed = 0;
        speedIntegrated_ = 0;
    }
    lastSpeed_ = robotSpeed_;

    /* Stepwise P-Controller for angle */
    const double P_ANGLE = 1;
    const double P_ANGLE2 = 0.6;
    const double P_ANGLE3 = 0.38;
    const double P_ANGLE4 = 0.2;


    if (distToFinalGoal < targetPoints_.front().Precision)
    {
        if (targetPoints_.front().Heading)
        {

            if (turn(targetPoints_.front(), true))
            {
                targetPoints_.erase(targetPoints_.begin());
            }
        }
        else
        {
            targetPoints_.erase(targetPoints_.begin());
        }
    }
    else if (fabs(angleDiff) > NO_DIRVE_ANGLE)
    {
        turn(goalSeg.getDirectionVector());
        speedIntegrated_ = 0;
        motorControllerSpeed = 0;
    }
    else
    {
        int velForward;
        double velDiff = 0;
        int timeMotor = 100;
        if ((distToFinalGoal < BRAKE_DISTANCE) && (targetPoints_.front().Brake == true))
        {
            speedIntegrated_ = 0;
			velForward = 45 + round(distance * 80 * desiredSpeed_);
            timeMotor = 50;
        }
        else
        {
            double angleFactor = 1 - (5 - rad2deg(fabs(angleDiff))) / 20.;
            constraint(angleFactor, 0.2, 1.);
            velForward = round(motorControllerSpeed * angleFactor);
        }

        if (fabs(rad2deg(angleDiff)) > 30)
        {
            velDiff =  ceil(P_ANGLE * rad2deg(angleDiff));
        }
        else if (fabs(rad2deg(angleDiff)) > 20)
        {
            velDiff =  ceil(P_ANGLE2 * rad2deg(angleDiff));
        }
        else if (fabs(rad2deg(angleDiff)) > 8)
        {
            velDiff =  ceil(P_ANGLE3 * rad2deg(angleDiff));
        }
        else
        {
            velDiff =  round(P_ANGLE4 * rad2deg(angleDiff));
        }

        if (driveBackwards)
        {
            velForward *= -1;
        }


#ifdef SIMULATION
        // ONLY SIMULATION
        MoveMs(velForward - round(velDiff), velForward + round(velDiff), 300, 0);
#else
        // Real Robots
        //MoveMs(velForward + round(velDiff), velForward - round(velDiff), 200, 50);

        MoveMs(velForward + round(velDiff), velForward - round(velDiff), timeMotor, 50);
#endif
    }
}

bool Agent::turn(const TargetPoint& tp, bool precise)
{
    bool result;
    double angleDiff = heading_.getAngle(*tp.Heading);
    angleDiff = (fabs(angleDiff) > M_PI/2)?(angleDiff - sign(angleDiff) * M_PI):angleDiff;
    result = fabs(angleDiff) < tp.AnglePrecision;

    if (result == false)
    {
        turn(*tp.Heading, precise);
    }

    return result;
}

bool Agent::turn(const Vector2d& dir, double precision)
{
    turn(dir);
    return heading_.getAngle(dir) < precision;
}

void Agent::turn(const Vector2d& dir, bool precise)
{
    const double P_ANGLE = (precise) ? 0.7 : 1.;

    double angleDiff = heading_.getAngle(dir);
    int timeToTurn = precise ? fabs(rad2deg(angleDiff) * 4) : fabs(rad2deg(angleDiff) * 6);

	constraint(timeToTurn, 10, 45);

    if (fabs(angleDiff) > M_PI/2)
    {
        angleDiff = angleDiff - sign(angleDiff) * M_PI ;
        driveBackwards = true;
    }

    double velDiff = P_ANGLE * rad2deg(angleDiff);

    if (sign(angleDiff) == 1)
    {
        constraint(velDiff, 25., 255.);
    }
    else
    {
        constraint(velDiff, -255., -25.);
    }

#ifdef SIMULATION
    MoveMs(-velDiff, velDiff, 300, 0);
#else
    MoveMs(velDiff, -velDiff, timeToTurn, 0);
#endif
}

void Agent::setTargetPoint(const TargetPoint& tp)
{
    targetPoints_.clear();
    targetPoints_.push_back(tp);

}

void Agent::addTargetPoint(const TargetPoint& tp)
{
    targetPoints_.push_back(tp);
}


void Agent::setTargetPoint(const Position& tp)
{
    targetPoints_.clear();
    targetPoints_.push_back(TargetPoint(tp));
}


void Agent::addTargetPoint(const Position& tp)
{
    targetPoints_.push_back(TargetPoint(tp));
}

void Agent::setTargetPoints(const std::vector<TargetPoint>& tp)
{
    targetPoints_.clear();
    targetPoints_ = tp;
}

bool Agent::isBeforBall(double val)
{
    static bool isInFront;

    double scalarRoboBall =  physics_->getSimpleBallTrajectory().getDirectionVector().getNormalized()* (    Vector2d(physics_->getBall()->GetPos())-position_ ).getNormalized();


    if(scalarRoboBall > val){//if true then robo is behind the ball
        isInFront = false;
    }else{
        isInFront = true;
    }
    return isInFront;
}
void Agent::startStopBall(){
	if(stopBallActive_)
		stopBallSM.changeState(STOPBALL_STATES::INIT);
	stopBallActive_ = true;
}

void Agent::stopBall()
{
	SUBSM_DURING(stopBallSM)
		case STOPBALL_STATES::INIT :
            if(physics_->getBallVelocity().getLength() < 0.15){
				stopBallSM.changeState(STOPBALL_STATES::NOT_MOVING_BALL);
			}else{
			if(isBehindBall(0.25)){
					stopBallSM.changeState(STOPBALL_STATES::BEFORE_BALL);
				}else{
					stopBallSM.changeState(STOPBALL_STATES::OVERTAKE_BALL);
				}
			}
			break;
		case STOPBALL_STATES::NOT_MOVING_BALL :
            if(physics_->getBallVelocity().getLength() > 0.15 || isAtTarget()){
                stopBallSM.changeState(STOPBALL_STATES::END);
            }
            break;
        case STOPBALL_STATES::OVERTAKE_BALL :
			if(isBehindBall(0.25)){
                stopBallSM.changeState(STOPBALL_STATES::BEFORE_BALL);
			}else{
				TargetPoint inBallDirection(physics_->getPredBallPosition(1000) , 0.05, true);
				setTargetPoint(inBallDirection);
            }
            break;
        case STOPBALL_STATES::BEFORE_BALL :
                if(physics_->getSimpleBallTrajectory().getDistance(position_) > 0.05){
                    TargetPoint nearestPoint(physics_->getSimpleBallTrajectory().getClosestPoint(position_), 0.05);
                    if(physics_->isInsideGameField(nearestPoint.Location.toPosition())){
                        setTargetPoint(nearestPoint);
                    }else{
                        cout << "TargetPoint not in GameField! Drive to ball psotion!" << endl;
                        stopBallSM.changeState(STOPBALL_STATES::BLOCK_BALL);
                    }
                }else{
                    stopBallSM.changeState(STOPBALL_STATES::BLOCK_BALL);
                }
            break;
        case STOPBALL_STATES::BLOCK_BALL :
            if(position_.getDistance(physics_->getBall()->GetPos()) > 0.05){
                setTargetPoint(TargetPoint(physics_->getBall()->GetPos(), 0.05));
                if(position_.getDistance(physics_->getBall()->GetPos()) < 0.15){
                    setDesiredSpeed(0.1);
                }
            }else{
                stopBallSM.changeState(STOPBALL_STATES::END);
            }    
            break;
        case STOPBALL_STATES::END :
            break;
    SUBSM_EXIT(stopBallSM)
            case STOPBALL_STATES::END :
                deleteTargetPoints();
                break;
    SUBSM_ENTRY(stopBallSM)
            case STOPBALL_STATES::INIT  :
                deleteTargetPoints();
                activate();
                setDesiredSpeed(1);
                cout << "STOPBALL: Roboter stopped!" << endl;
                break;
            case STOPBALL_STATES::NOT_MOVING_BALL  :
                if(ourSide_ == RIGHT_SIDE){
                    setTargetPoint( Vector2d(physics_->getBall()->GetPos().GetX(), physics_->getBall()->GetPos().GetY()+0.20 ));
                    addTargetPoint( Vector2d(physics_->getBall()->GetPos().GetX()+0.15, physics_->getBall()->GetPos().GetY() ));
                }else{
                    setTargetPoint( Vector2d(physics_->getBall()->GetPos().GetX(), physics_->getBall()->GetPos().GetY()+0.20 ));
                    addTargetPoint( Vector2d(physics_->getBall()->GetPos().GetX()-0.15, physics_->getBall()->GetPos().GetY() ));
                }
                cout << "STOPBALL: Ball is not moving, cruise to ball!" << endl;
                break;
            case STOPBALL_STATES::OVERTAKE_BALL :
                cout << "STOPBALL: Roboter is behind the ball!" << endl;
                break;
            case STOPBALL_STATES::BEFORE_BALL :
                cout << "STOPBALL: Roboter is before ball cruise to ball trajectory!" << endl;
                break;
            case STOPBALL_STATES::BLOCK_BALL :
                cout << "STOPBALL: Stopping Ball!" << endl;
                break;
            case STOPBALL_STATES::END :
                cout << "STOPBALL: Ball stoped!" << endl;
                break;
    SUBSM_END(stopBallSM)
}

void Agent::stopAllActions()
{
    attackerModeActive_ = false;
    supportGkActive_ = false;
    shootBallActive_ = false;
    clearBallActive_ = false;
    stopBallActive_ = false;
    passToActive_ = false;
}

void Agent::startClearBall()
{
    stopAllActions();
    clearBallSM.changeState(CLEARBALL_STATES::INIT);
    clearBallActive_ = true;
}

void Agent::clearBall()
{
    SUBSM_DURING(clearBallSM)
        case CLEARBALL_STATES::INIT  :
			if( isBehindBall(0.2) ){
                clearBallSM.changeState(CLEARBALL_STATES::CLEAR);
            }else{
                clearBallSM.changeState(CLEARBALL_STATES::STOP_BALL);
            }
            break;
        case CLEARBALL_STATES::STOP_BALL :
		if(isBehindBall(0.15) || (isBehindBall(0) && physics_->getBallVelocity().getLength() < 0.1)){
				clearBallSM.changeState(CLEARBALL_STATES::CLEAR);
			}else{
			TargetPoint behindBall = TargetPoint(Line(physics_->getBallPositionFiltered(), ownGoalSegment_->getMiddlePoint()).getDirectionVector().getNormalized()*0.1);
				setTargetPoint(behindBall);
			}


//			double OvertakeY, OvertakeX;


//			if(physics_->getSimpleBallTrajectory().isLeftOfLine(position_.toPosition())){
//				OvertakeY = physics_->getBallLastPosition().GetY()-0.25;
//			}else{
//				OvertakeY = physics_->getBallLastPosition().GetY()+0.25;
//			}

//			if(ourSide_ == RIGHT_SIDE){
//				OvertakeX =  physics_->getBallLastPosition().GetX()+0.30;
//			}else{
//				OvertakeX =  physics_->getBallLastPosition().GetX()-0.30;
//			}

//			if(physics_->isInsideGameField(  Vector2d(OvertakeX, OvertakeY).toPosition()  ) ){
//				setTargetPoint(TargetPoint( Vector2d(OvertakeX, OvertakeY) , 0.05, true));
//			}else{
//				setTargetPoint(TargetPoint( physics_->getPredBallPosition(500) , 0.05, true));
//			}
//			if(isBehindBall(0.2)){
//				clearBallSM.changeState(CLEARBALL_STATES::CLEAR);
//			}
            break;
        case CLEARBALL_STATES::CLEAR :
			if(isBehindBall(0)){
				if(position_.getDistance(physics_->getBall()->GetPos()) > 0.20){
					setDesiredSpeed(0.6);
				}else{
					setDesiredSpeed(1.5);
				}
				TargetPoint aim = TargetPoint(physics_->getBall()->GetPos(), 0.05,  false);
				avoidPenaltyZone(&aim);
				setTargetPoint(aim);
            }else{
                clearBallSM.changeState(CLEARBALL_STATES::INIT);
            }
            break;
        case CLEARBALL_STATES::END :
            clearBallSM.changeState(CLEARBALL_STATES::INIT);
        break;

    SUBSM_EXIT(clearBallSM)
		case CLEARBALL_STATES::STOP_BALL :
			deactivateCA();
		case CLEARBALL_STATES::END :
            deleteTargetPoints();
            break;
    SUBSM_ENTRY(clearBallSM)
            case CLEARBALL_STATES::INIT  :
                deleteTargetPoints();
				setDesiredSpeed(1.0);
				cout << "CLEARBALL: INIT!" << endl;
                break;
            case CLEARBALL_STATES::STOP_BALL :
				activateCA(false, false, true, true, true, true);
                break;
            case CLEARBALL_STATES::CLEAR :
                break;
            case CLEARBALL_STATES::END :
                break;
    SUBSM_END(clearBallSM)
}

bool Agent::isBehindBall(double val){
	// OURSIDE = RIGHT_SIDE
	if (ourSide_ == RIGHT_SIDE){
		if (GetPos().GetX()- val > physics_->getBall()->GetX()){
			return true;
		}
		else {
			return false;
		}
	}
	else{
		if (GetPos().GetX()+ val < physics_->getBall()->GetX()){
			return true;
		}
		else {
			return false;
		}
	}
}

bool Agent::isGoodBehindBall(){
    Vector2d ballPos = Vector2d(physics_->getBall()->GetPos());
    Vector2d target = enemyGoalSegment_->getMiddleVector();
    LineSegment goalBall(ballPos, target);
    Vector2d normVec = goalBall.getNormalVector().getNormalized();
    Vector2d dirVec = goalBall.getDirectionVector().getNormalized();
    Vector2d p2 = Vector2d(ballPos - dirVec * 0.6);
    Vector2d p3 = Vector2d(ballPos - dirVec * 0.6 + normVec * 0.3);
    Vector2d p1 = Vector2d(ballPos - dirVec * 0.6 - normVec * 0.3);

    return Quadrangle(ballPos.toPosition(),p1.toPosition(),p2.toPosition(),p3.toPosition()).isInside(position_.toPosition());
}


Line Agent::getBallGoalLine(){
    return Line(physics_->getBall()->GetPos(), enemyGoalSegment_->getMiddlePoint());
}

void Agent::avoidPenaltyZone(TargetPoint* target){
    if (enemyPenaltyZone_->isInside(target->Location.toPosition()))
    {
        //cout << "ball is in enemyPenaltyZone, getValidPosition" << endl;
        target->Location = enemyPenaltyZone_->getValidPosition(GetPos());
    }
    if (ownPenaltyZone_->isInside(target->Location.toPosition()))
    {
        //cout << "ball is in enemyPenaltyZone, getValidPosition" << endl;
        target->Location = ownPenaltyZone_->getValidPosition(GetPos());
    }
}

double Agent::timeToBall(){
    double t = 0;
    if (robotSpeed_ != 0)
    {
    t = (position_.getDistance(physics_->getBall()->GetPos()) / robotSpeed_) * 1000;
    }
    if ( t > 3000)
    {
        t = 3000;
    }
    return t;
}

void Agent::startShootBall(){
    stopAllActions();
    shootBallActive_=true;
}

void Agent::shootBall(){

    double offsetX = 0.25;
    double offsetY = 0.2;
    TargetPoint aim;

    const int shotDuration = 700;
    const double shootDist = 0.15;
    static int shootTimer = shotDuration;
    static int predTime = 3000;

    if (ourSide_ == LEFT_SIDE)
    {
        offsetX *= -1;
    }
    if (leftSide_->isInside(physics_->getBallLastPosition()))
    {
        offsetY *= -1;
    }


    SUBSM_DURING(shootBallSM)
          case SHOOTBALL_STATES::INIT :
                if (isBehindBall())
                {
                    shootBallSM.changeState(SHOOTBALL_STATES::GET_ON_BALL_GOAL_LINE);
                }
                else{
                    shootBallSM.changeState(SHOOTBALL_STATES::GET_BEHIND_BALL);
                }
           break;
           case SHOOTBALL_STATES::GET_BEHIND_BALL :
                if (!isBehindBall(0.2))
                {
                    Position predBallPos = physics_->getPredBallPosition(3000);
                    aim = TargetPoint(Vector2d(predBallPos.GetX()+offsetX,predBallPos.GetY()),0.01,false);

                    avoidPenaltyZone(&aim);
                    setTargetPoint(aim);
                }
                else
                {
                    shootBallSM.changeState(SHOOTBALL_STATES::GET_ON_BALL_GOAL_LINE);
                }
           break;
            case SHOOTBALL_STATES::GET_ON_BALL_GOAL_LINE :
                if (isBehindBall())
                {
                    aim = calcShootPosition(physics_->getPredBallPosition(predTime), enemyGoalSegment_->getMiddlePoint(), shootDist, true);
                    if (predTime > shotDuration)
                        predTime -= 30;

                    if (!physics_->getField().isInside(aim.Location.toPosition()))
                    {
                        // if ball is stuck on the fieldborder shoot it to free it
                        // cout << "pos is outside field" << endl;
                        shootBallSM.changeState(SHOOTBALL_STATES::SHOOT_BALL);
                    }
                    avoidPenaltyZone(&aim);
                    setTargetPoint(aim);

                    if (position_.getDistance(aim.Location) < 0.05 && predTime <= shotDuration)
                    {
                        shootBallSM.changeState(SHOOTBALL_STATES::SHOOT_BALL);
                    }
                }
                else
                {
                    shootBallSM.changeState(SHOOTBALL_STATES::GET_BEHIND_BALL);
                }
            break;
            case SHOOTBALL_STATES::SHOOT_BALL :
                if (isBehindBall() && !enemyPenaltyZone_->isInside(physics_->getPredBallPosition(shootTimer)))
                {
                    aim = TargetPoint(physics_->getPredBallPosition(shootTimer),0.01,false);
                    setTargetPoint(aim);

                    shootTimer -= 30;
                    if (shootTimer <= 0)
                    {
                        shootTimer = 0;
                    }
                    //shoot();
                }
                else{
                    shootBallSM.changeState(SHOOTBALL_STATES::INIT);
                }
            break;
            case SHOOTBALL_STATES::END :
            break;
    SUBSM_EXIT(shootBallSM)
    SUBSM_ENTRY(shootBallSM)
               case SHOOTBALL_STATES::INIT :
                   setDesiredSpeed(0.8);
                   cout << "start shoot sequence" << endl;
                   break;
               case SHOOTBALL_STATES::GET_BEHIND_BALL :
                   cout << "Roboter gets behind ball" << endl;
                   // ( enemies, agents, ownPenaltyZone, enemyPenaltyZone, ballObst, gameField)
                   activateCA(true, true, true, true, true, true);
                   break;
               case SHOOTBALL_STATES::GET_ON_BALL_GOAL_LINE :
                   cout << "Roboter gets on Ball Goal Line" << endl;
                   activateCA(false, false, true, true, false, false);
                   predTime = 3000;
                   break;
               case SHOOTBALL_STATES::SHOOT_BALL :
                   cout << "Roboter shoots Ball"<< endl;
                   setDesiredSpeed(1.2);
                   shootTimer = shotDuration;
                   activateCA(false, false, true, true, false, false);
                   break;
               case SHOOTBALL_STATES::END :
                   cout << "Ball was successfully shot" << endl;
                   break;
    SUBSM_END(shootBallSM)
}

Line Agent::getBallTargetLine(Position target){
    return Line(physics_->getPredBallPosition(300),target);
}

void Agent::startPassTo(Agent *agent){
    passToAgent_ = agent;
}

void Agent::passTo()
{
    TargetPoint aim;
    Position agent;

    double offsetX = 0.20;
    double offsetY = 0.10;
    if (ourSide_ == LEFT_SIDE)
    {
        offsetX *= -1;
    }
    if (leftSide_->isInside(physics_->getBallLastPosition()))
    {
        offsetY *= -1;
    }

    SUBSM_DURING(passToSM)
            case PASSTO_STATES::INIT :
                if (isBehindBall())
                {
                    passToSM.changeState(PASSTO_STATES::GET_ON_BALL_TARGET_LINE);
                }
                else{
                    passToSM.changeState(PASSTO_STATES::GET_BEHIND_BALL);
                }
            break;
            case PASSTO_STATES::GET_BEHIND_BALL :
                if (!isBehindBall(0.15))
                {
                    Position predBallPos = physics_->getPredBallPosition(200);
                    aim = TargetPoint(Vector2d(predBallPos.GetX()+offsetX,predBallPos.GetY()),0.01,false);

                    setTargetPoint(aim);
                }
                else
                {
                    shootBallSM.changeState(PASSTO_STATES::GET_ON_BALL_TARGET_LINE);
                }
            break;
            case PASSTO_STATES::GET_ON_BALL_TARGET_LINE :
               agent = passToAgent_->GetPos();
               agent = Position(agent.GetX()+offsetX,agent.GetY());
               cout << "agent pos: " << agent << endl;
               aim = calcShootPosition(physics_->getPredBallPosition(100),agent, 0.15,true);
               setTargetPoint(aim);

               if (position_.getDistance(aim.Location) < 0.05 )
               {
                   passToSM.changeState(PASSTO_STATES::PASS_BALL);
               }
            break;
            case PASSTO_STATES::PASS_BALL :
                if (isBehindBall())
                {
                    aim = TargetPoint(physics_->getPredBallPosition(100),0.01,false);
                    setTargetPoint(aim);
                }
                else
                {
                   passToSM.changeState(PASSTO_STATES::INIT);
                }
            break;

    SUBSM_EXIT(passToSM)
    SUBSM_ENTRY(passToSM)
            case PASSTO_STATES::INIT :
                setDesiredSpeed(0.7);
                // ( enemies, agents, ownPenaltyZone, enemyPenaltyZone, ballObst, gameField)
                cout << "start pass sequence" << endl;
                break;
            case PASSTO_STATES::GET_BEHIND_BALL :
                cout << "Roboter gets behind ball" << endl;
                // ( enemies, agents, ownPenaltyZone, enemyPenaltyZone, ballObst, gameField)
                //activateCA(true, true, true, true, true, true);
                break;
            case PASSTO_STATES::GET_ON_BALL_TARGET_LINE :
                // activateCA(true, true, true, true, false, true);
                 cout << "Roboter gets on Ball Target Line" << endl;
                break;
            case PASSTO_STATES::PASS_BALL :
                setDesiredSpeed(1.2);
                // activateCA(false,false,true,true,false,false);
                cout << "Roboter passes Ball"<< endl;
                break;

    SUBSM_END(passToSM)
}


Vector2d Agent::calcRelativePosition(double x, double distance_left_or_right, eSide left_or_right)
{
    if (ourSide_ == LEFT_SIDE)
    {
        if (left_or_right == LEFT_SIDE)
        {
            return Vector2d(x,-distance_left_or_right);
        }
        else
        {
            return Vector2d(x,distance_left_or_right);
        }
    }
    else
    {
        if (left_or_right == LEFT_SIDE)
        {
            return Vector2d(x,distance_left_or_right);
        }
        else
        {
            return Vector2d(x,-distance_left_or_right);
        }
    }
}



LineSegment Agent::calcPositionBeforeBall(Position ball, Position target, double distToBall)
{
    LineSegment seg(ball,target);
    //result.angle = Angle(seg.getDirectionVector().getAngle());
    LineSegment result = LineSegment(seg.getStartVector() - seg.getDirectionVector() * distToBall, target);
    return result;
}

TargetPoint Agent::calcShootPosition(Position ball, Position target, double distToBall, bool brake)
{
    LineSegment seg(ball,target);
    //result.angle = Angle(seg.getDirectionVector().getAngle());
    LineSegment result = LineSegment(seg.getStartVector() - seg.getDirectionVector() * distToBall, target);

    TargetPoint targetPoint(result.getStartVector(), result.getDirectionVector(), 0.08, brake);
    return targetPoint;
}

void Agent::shoot()
{
    double angle = heading_.getAngle(Line(position_, physics_->getBallPositionFiltered()).getDirectionVector());
    bool kickBackwards = fabs(angle) > M_PI_2;
    noWarning_ = true;
    if (kickBackwards)
        MoveMsBlocking(-255,-255,300,100);
    else
        MoveMsBlocking(255,255,300,100);
}

void Agent::activateCA(bool enemies, bool agents, bool ownPenaltyZone, bool enemyPenaltyZone, bool ballObst, bool gameField)
{
    useCA_ = true;
    useGameField_ = gameField;
    obstacles_.clear();
    if (agents)
    {
        int i = 0;
        for (auto k : physics_->getAgentObstacles())
        {
            if (physics_->getAgent(i)->getId() != getId())
                obstacles_.push_back(k);
            i++;
        }
    }
    if (enemies)
    {
        for (auto k : physics_->getEnemyObstacles())
        {
            obstacles_.push_back(k);
        }
    }
    if (ownPenaltyZone)
    {
        if (ourSide_ == LEFT_SIDE)
        {
            obstacles_.push_back(physics_->getObstaclePenaltyAreaLeftPtr());
        }
        else
        {
            obstacles_.push_back(physics_->getObstaclePenaltyAreaRightPtr());
        }
    }
    if (enemyPenaltyZone)
    {
        if (ourSide_ == LEFT_SIDE)
        {
            obstacles_.push_back(physics_->getObstaclePenaltyAreaRightPtr());
        }
        else
        {
            obstacles_.push_back(physics_->getObstaclePenaltyAreaLeftPtr());
        }
    }
    if (ballObst)
    {
        obstacles_.push_back(physics_->getBallObstacle());
    }
}









