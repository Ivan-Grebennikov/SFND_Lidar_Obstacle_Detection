#include "helpers.h"
#include <cmath>

int randomOnInterval(int _interval)
{
    return std::lround (((double)rand() / RAND_MAX) * _interval);
}

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
    :	point(arr), id(setId), left(NULL), right(NULL)
    {}
};

KdTree3D::KdTree3D()
    : root(NULL)
{}

void KdTree3D::insert(std::vector<float> point, int id)
{
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the root

    if (!root) {
        root = new Node(point, id);
    }
    else {
        insert(&root, 0, point, id);
    }
}

std::vector<int> KdTree3D::search(std::vector<float> target, float distanceTol)
{
    std::vector<int> ids;

    search(ids, 0, root, target, distanceTol);

    return ids;
}

enum KDTree3DSplitting
{
    KDT3DS_X = 0,
    KDT3DS_Y = 1,
    KDT3DS_Z = 2,
    KDT3DS_Count
};

bool box3DContains(const std::vector<float> &_center, float _side, const std::vector<float> &_point)
{
    return _center[KDT3DS_X] - _side <= _point[KDT3DS_X] && _center[KDT3DS_X] + _side >= _point[KDT3DS_X] &&
            _center[KDT3DS_Y] - _side <= _point[KDT3DS_Y] && _center[KDT3DS_Y] + _side >= _point[KDT3DS_Y] &&
            _center[KDT3DS_Z] - _side <= _point[KDT3DS_Z] && _center[KDT3DS_Z] + _side >= _point[KDT3DS_Z];
}

float euclideanDistance3D(const std::vector<float> &_first, const std::vector<float> &_second)
{
    return std::sqrt(std::pow(_first[KDT3DS_X] - _second[KDT3DS_X], 2) +
                     std::pow(_first[KDT3DS_Y] - _second[KDT3DS_Y], 2) +
                     std::pow(_first[KDT3DS_Z] - _second[KDT3DS_Z], 2));
}

void KdTree3D::insert(Node **_node, int _depth, const std::vector<float> &_point, int _id)
{
    if (!(*_node))
    {
        *_node = new Node(_point, _id);
    }
    else
    {
        _depth += 1;

        const KDTree3DSplitting splitting = (KDTree3DSplitting)(_depth % KDT3DS_Count);

        if (_point[splitting] < (*_node)->point[splitting])
        {
            insert(&((*_node)->left), _depth, _point, _id);
        }
        else
        {
            insert(&((*_node)->right), _depth, _point, _id);
        }
    }
}

void KdTree3D::search(std::vector<int> &_ids, int _depth, Node *_node, const std::vector<float> &_target, float _distance)
{
    if (!_node)
    {
        return;
    }
    else
    {
        _depth += 1;

        if (box3DContains(_target, _distance, _node->point))
        {
            if (euclideanDistance3D(_target, _node->point) <= _distance)
            {
                _ids.push_back(_node->id);
            }

            search(_ids, _depth, _node->left, _target, _distance);
            search(_ids, _depth, _node->right, _target, _distance);
        }
        else
        {
            const KDTree3DSplitting splitting = (KDTree3DSplitting)(_depth % KDT3DS_Count);

            if (_target[splitting] - _distance <= _node->point[splitting] && _target[splitting] + _distance >= _node->point[splitting])
            {
                search(_ids, _depth, _node->left, _target, _distance);
                search(_ids, _depth, _node->right, _target, _distance);
            }
            else if (_target[splitting] + _distance < _node->point[splitting])
            {
                search(_ids, _depth, _node->left, _target, _distance);
            }
            else
            {
                search(_ids, _depth, _node->right, _target, _distance);
            }
        }
    }
}

void proximity(const std::vector<std::vector<float> > &_points, int _current, std::vector<int> &_cluster, std::unordered_set<int> &_processed, KdTree3D *_tree, float _distanceTol)
{
    _processed.insert(_current);
    _cluster.push_back(_current);

    const std::vector<int> neighbours = _tree->search(_points[_current], _distanceTol);

    for (int neighbour : neighbours)
    {
        if (_processed.find(neighbour) == std::end(_processed)) {

            proximity(_points, neighbour, _cluster, _processed, _tree, _distanceTol);
        }
    }
}

std::vector<std::vector<int> > euclideanCluster(const std::vector<std::vector<float> > &_points, KdTree3D *_tree, float _distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;

    std::unordered_set<int> processed;

    for (int point = 0; point < _points.size(); ++point) {

        if (processed.find(point) != std::end(processed))
            continue;

        clusters.emplace_back();

        proximity(_points, point, clusters.back(), processed, _tree, _distanceTol);
    }

    return clusters;
}
