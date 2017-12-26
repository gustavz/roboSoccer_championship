#include "Node.h"


Node::Node(const Vector2d &position, bool inObstacle, const Node *previous, unsigned int axis, Node *parent) :
    position_(position),
    inObstacle_(inObstacle),
    previous_(previous),
    axis_(axis),
    parent_(parent)
{
    child_[0] = NULL;
    child_[1] = NULL;
}

Node::~Node()
{
    delete child_[0];
    delete child_[1];
}


Node** Node::nearestChildPointer(const Vector2d &position)
{
    return &child_[position[axis_] > position_[axis_]];
}

Node* Node::nearestChild(const Vector2d &position) const
{
    return child_[position[axis_] > position_[axis_]];
}

Node* Node::farthestChild(const Vector2d &position) const
{
    return child_[position[axis_] <= position_[axis_]];
}

void Node::getChildren(std::vector<const Node*> &nodes)
{
    for (int i = 0; i < 2; i++) {
        if (child_[i]) {
            nodes.push_back(child_[i]);
            child_[i]->getChildren(nodes);
        }
    }
}

unsigned int Node::depth() const
{
    unsigned int d = 0;
    if (child_[0]) {
        d = child_[0]->depth();
    }

    if (child_[1]) {
        d = std::max(d, child_[1]->depth());
    }

    return d + 1;
}
