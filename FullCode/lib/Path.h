#ifndef PATH_H
#define PATH_H

#include "list"
#include "Geometry.h"
#include "helper/KdTree.h"

#include <boost/optional.hpp>

/**
 *@brief defines target point and how it is supposed to be reached
 */
struct TargetPoint
{
    Vector2d Location;                  /**< position of target point */
    boost::optional<Vector2d> Heading;  /**< optional heading direction */
    double Precision = 0.1;             /**< precision */
    double AnglePrecision = deg2rad(5.);       /**< angle precision */
    bool Brake = true;                  /**< status if braking is activated */

    /**
     *@brief standard constructor
     */
    TargetPoint(){}

    /**
     *@brief constructor of target point with position
     *@param pos: target position
     *@param robBrake: braking active
     */
    TargetPoint(Position pos, bool robBrake = true)
        :
          Location(pos),
          Brake(robBrake)
    {
    }

    /**
     *@brief constructor of target point with position and heading
     *@param pos: target position
     *@param heading: target heading
     *@param robBrake: braking active
     */
    TargetPoint(Position pos, Vector2d heading, bool robBrake = true)
        :
          Location(pos),
          Heading(heading),
          Brake(robBrake)
    {}

    /**
     *@brief constructor of target point with position and precision
     *@param pos: target position
     *@param targetPrecision: target precision
     *@param robBrake: braking active
     */
    TargetPoint(Position pos, double targetPrecision, bool robBrake = true)
        :
          Location(pos),
          Precision(targetPrecision),
          Brake(robBrake)
    {}

    /**
     *@brief constructor of target point with position, heading and precision
     *@param pos: target position
     *@param heading: target heading
     *@param targetPrecision: target precision
     *@param robBrake: braking active
     */
    TargetPoint(Position pos, Vector2d heading, double targetPrecision, bool robBrake = true)
        :
          Location(pos),
          Heading(heading),
          Precision(targetPrecision),
          Brake(robBrake)
    {}

    /**
     *@brief constructor of target point with position
     *@param pos: target position
     *@param robBrake: braking active
     */
    TargetPoint(Vector2d pos, bool robBrake = true)
        :
          Location(pos),
          Brake(robBrake)
    {
    }

    /**
     *@brief constructor of target point with position and heading
     *@param pos: target position
     *@param heading: target heading
     *@param robBrake: braking active
     */
    TargetPoint(Vector2d pos, Vector2d heading, bool robBrake = true)
        :
          Location(pos),
          Heading(heading),
          Brake(robBrake)
    {}

    /**
     *@brief constructor of target point with position and precision
     *@param pos: target position
     *@param targetPrecision: target precision
     *@param robBrake: braking active
     */
    TargetPoint(Vector2d pos, double targetPrecision, bool robBrake = true)
        :
          Location(pos),
          Precision(targetPrecision),
          Brake(robBrake)
    {}

    /**
     *@brief constructor of target point with position, heading and precision
     *@param pos: target position
     *@param heading: target heading
     *@param targetPrecision: target precision
     *@param robBrake: braking active
     */
    TargetPoint(Vector2d pos, Vector2d heading, double targetPrecision, bool robBrake = true)
        :
          Location(pos),
          Heading(heading),
          Precision(targetPrecision),
          Brake(robBrake)
    {}
};

class Physics;

class Path
{
  public:
    /**
     *@brief constructor of class Path
     *@param physics: pointer to physics
     *@param id: roboter ID
     */
    Path(Physics* physics, int id);

    /**
     *@brief default destructor
     */
    ~Path();

    /**
     *@brief computes path to target point
     *@param requestedEnd: desired target point
     */
    void compute(TargetPoint requestedEnd);

    /**
     *@brief initializes path and obstacles
     */
    void initializePath();

    /**
     *@brief computes trajectory based on tangent tollision avoidance
     *@param start: start position
     *@param end: position
     */
    std::vector<Position> computeLegacy(Position start, Position end);

    /**
     *@brief Getter for rfcomm id
     */
    int getId() {return id_;}

  protected:
    Physics* physics_;                      /**< pointer to physics */

    std::vector<TargetPoint> targetPoints_; /**< vector of target points */
    std::vector<TargetPoint> CAtargetPoints_; /**< vector of waypoints of collision avoidance trajectory */
    std::vector<TargetPoint> cachedCAtargetPoints_; /**< vector of cached waypoints of collision avoidance trajectory */
    Vector2d position_;                     /**< position */

    int id_; /**< roboter ID */

    std::vector<Obstacle*> obstacles_;  /**< vector of obstacles */

    bool useGameField_ = true; /**< Use Game Field Obstacle in collision avoidance */

  private:

    Quadrangle* field_;                 /**< quadrangle defining field */
    std::vector<Quadrangle*> corners_;  /**< vector of corner obstacles */

    /**
     *@brief determines if line segment intersects with obstacle
     *@param seg: line segment
     */
    bool intersectsObstacle(const LineSegment& seg) const;

    /**
     *@brief determines if line segment intersects with obstacle
     *@param seg: line segment
     *@param idx: index of obstacle that shall be ignored
     */
    bool intersectsObstacle(const LineSegment& seg, int idx) const;


    KdTree *tree_; /**< KdTree object for RRT pathfinding */
    const double p_dest_ = 0.5; /**< probability of extending node towards target */
    double stepSize_ = 0.04; /**< step size of node extension in meter */
    const int nr_iterations_ = 350; /**< maximum number of tree extend iterations*/
    const int nr_pp_steps_ = 2; /**< number of path post-processing iterations */

    /**
     *@brief returns random vector inside field
     */
    Vector2d randomState() const;

    /**
     *@brief tries to expand KdTree, returns Node* if successful, returns NULL if unsuccessful
     *@param *fromNode: Node to extend from
     *@param &to: extend towards direction
     */
    const Node* extend(const Node *fromNode, const Vector2d &to);

    /**
     *@brief cuts out redundant nodes in generated path
     */
    void simplify();

    /**
     *@brief processes path to straighten out jagged waypoints
     */
    void cutCorners();


};



#endif // PATH_H
