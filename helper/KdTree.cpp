#include "KdTree.h"

KdTree::KdTree(const Vector2d &position, bool inObstacle) :
    nodeCount_(1)
{
    root_ = new Node(position, inObstacle, NULL, 0, NULL);
}

KdTree::~KdTree()
{
    delete root_;
}


Node* KdTree::insert(const Vector2d &position, bool inObstacle, const Node *previous)
{
    Node *parent = NULL;
    Node **next = &root_;

    unsigned int axis;
    do {
        axis = (*next)->axis();
        parent = (*next);
        next = parent->nearestChildPointer(position);
    } while (*next);

    *next = new Node(position, inObstacle, previous, axis ^ 1, parent);
    nodeCount_++;

    return *next;
}

const Node* KdTree::nearest(const Vector2d &position) const
{
    double bestDist = INFINITY;
    double bestDistSquared = INFINITY;
    return nearest(position, root_, bestDist, bestDistSquared, NULL);
}


Node* KdTree::nearest(const Vector2d &position, Node *root, double &bestDist, double &bestDistSquared, Node *bestNode) const
{
    if (!root) {
        return bestNode;
    }

    Node *currentNode = NULL;

    {
        Node *node = root;
        do {
            currentNode = node;
            node = node->nearestChild(position);
        } while (node);
    }

    do {
        const double dist = (currentNode->position() - position).getLengthSquared();
        if (dist < bestDistSquared) {
            bestDistSquared = dist;
            bestDist = std::sqrt(dist);
            bestNode = currentNode;
        }

        const unsigned int axis = currentNode->axis();
        if (std::abs(position[axis] - currentNode->position()[axis]) <= bestDist) {
            bestNode = nearest(position, currentNode->farthestChild(position), bestDist, bestDistSquared, bestNode);
        }

        if (currentNode == root) {
            break;
        }

        currentNode = currentNode->parent();
    } while (currentNode);

    return bestNode;
}


unsigned int KdTree::depth() const
{
    return root_->depth();
}

const Vector2d KdTree::position(const Node *node) const
{
    return node->position();
}

bool KdTree::inObstacle(const Node *node) const
{
    return node->inObstacle();
}

const Node* KdTree::previous(const Node *node) const
{
    return node->previous();
}

const std::vector<const Node*> KdTree::getChildren() const
{
    std::vector<const Node*> nodes;
    root_->getChildren(nodes);
    return nodes;
}
