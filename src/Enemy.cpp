#include "lib/Enemy.h"
#include "lib/Physics.h"


double Enemy::getDistToBall()
{
    return physics_->getBallPositionFiltered().getDistance(GetPos());
}
