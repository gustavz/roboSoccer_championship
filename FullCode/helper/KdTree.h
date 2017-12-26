#ifndef KDTREE_H
#define KDTREE_H

#include "Node.h"

class KdTree
{
public:

    /**
     *@brief KdTree constructor
     *@param position: position of root node
     *@param inObstacle: root node inObstacle attribute
     */
    KdTree(const Vector2d &position, bool inObstacle);

    /**
     *@brief KdTree deconstructor
     */
    ~KdTree();

public:

    /**
     *@brief Inserts a new node into the tree
     *@param position: new node position
     *@param inObstacle: new node inObstacle attribute
     *@param previous: predecessor of new node
     */
    Node* insert(const Vector2d &position, bool inObstacle, const Node *previous);

    /**
     *@brief Returns nearest node in tree to position argument
     *@param position: position to find nearest node from
     */
    const Node* nearest(const Vector2d &position) const;

    /**
     *@brief Returns depth of tree
     */
    unsigned int depth() const;

    /**
     *@brief Returns number of nodes in tree
     */
    unsigned int nodeCount() const { return nodeCount_; }

    /**
     *@brief Returns root of tree
     */
    const Node* root() const { return root_; }

    /**
     *@brief Returns position of node
     *@param: node: node to get position from
     */
    const Vector2d position(const Node *node) const;


    /**
     *@brief Returns inObstacle attribute of node
     *@param: node: node to get inObstacle attribute from
     */
    bool inObstacle(const Node *node) const;


    /**
     *@brief Returns predecessor node of node
     *@param: node: node to get predecessor
     */
    const Node* previous(const Node *node) const;


    /**
     *@brief Returns all children of root
     */
    const std::vector<const Node*> getChildren() const;

private:

    /**
     *@brief Called by nearest(const Vector2d &position), finds nearest node to position
     *@param: position: position to find nearest node from
     *@param: root: root of tree to be searched
     *@param: bestDist: smallest distance found
     *@param: bestDistSquared: smallest distance squared found
     *@param: bestNode: closest node found, returned at end of method
     */
    Node* nearest(const Vector2d &position, Node *root, double &bestDist, double &bestDistSquared, Node *bestNode) const;

private:
    Node* root_; /**< root of KdTree */
    unsigned int nodeCount_; /**< number of nodes in tree */
};

#endif // KDTREE_H
