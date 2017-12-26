#ifndef NODE_H
#define NODE_H

#include "lib/Geometry.h"

class Node
{
public:

    /**
     *@brief Standard constructor for Node
     *@param position: position of new node
     *@param inObstacle: inObstacle attribute of new node
     *@param previous: predecessor node of new node
     *@param axis: axis that new node divides
     *@param parent: parent node of new node
     */
    Node(const Vector2d &position, bool inObstacle, const Node *previous, unsigned int axis, Node *parent);

    /**
     *@brief Deconstructor for Node
     */
    ~Node();

    /**
     *@brief Returns pointer to nearest child to position
     *@param position: position argument
     */
    Node** nearestChildPointer(const Vector2d &position);

    /**
     *@brief Return nearest child to position
     *@param position: position argument
     */
    Node* nearestChild(const Vector2d &position) const;

    /**
     *@brief Returns farthest child to position
     *@param position: position argument
     */
    Node* farthestChild(const Vector2d &position) const;

    /**
     *@brief Returns position of node
     */
    const Vector2d& position() const { return position_; }


    /**
     *@brief Returns inObstacle attribute of node
     */
    bool inObstacle() const { return inObstacle_; }

    /**
     *@brief Returns predecessor node
     */
    const Node* previous() const { return previous_; }

    /**
     *@brief Returns dividing axis of node
     */
    unsigned int axis() const { return axis_; }

    /**
     *@brief Returns parent node
     */
    Node* parent() const { return parent_; }

    /**
     *@brief Returns child defined by index
     *@param index: index argument
     */
    Node* child(unsigned int index) const { return child_[index]; }

    /**
     *@brief Returns depths of tree starting from this node
     */
    unsigned int depth() const;

    /**
     *@brief Returns all children of node
     *@param nodes: pointer to vector to save all children
     */
    void getChildren(std::vector<const Node*> &nodes);

private:
    const Vector2d position_; /**< position of node */
    const bool inObstacle_; /**< inObstacle attribute of node */
    const Node* const previous_; /**< predecessor node */

    const unsigned int axis_; /**< dividing axis of node */
    Node* const parent_; /**< parent node */
    Node* child_[2]; /**< children nodes */
};

#endif // NODE_H
