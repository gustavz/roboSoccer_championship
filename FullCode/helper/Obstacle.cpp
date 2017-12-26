#include "Obstacle.h"

std::ostream& operator<<(std::ostream& os, const Obstacle& obst)
{
    os << "Obstacle: " << obst.getName() << " at: " << Vector2d(obst.getCenter()) << std::endl;
    return os;
}
