#ifndef HELPERS_H_
#define HELPERS_H_

#include <cmath>
#include <vector>
#include <unordered_set>

int randomOnInterval(int _interval);

template <class PointT>
float distanceToLine(const PointT& _p1, const PointT& _p2, const PointT& _from)
{
    const float A = _p1.y - _p2.y;
    const float B = _p2.x - _p1.x;
    const float C = _p1.x*_p2.y - _p2.x*_p1.y;

    return std::abs(A*_from.x + B*_from.y + C) / std::sqrt(A*A + B*B);
}

template <class PointT>
float distanceToPlane(const PointT& _p1, const PointT& _p2,
                      const PointT& _p3, const PointT& _from)
{
    const float A = (_p2.y - _p1.y)*(_p3.z - _p1.z) - (_p2.z - _p1.z)*(_p3.y - _p1.y);
    const float B = (_p2.z - _p1.z)*(_p3.x - _p1.x) - (_p2.x - _p1.x)*(_p3.z - _p1.z);
    const float C = (_p2.x - _p1.x)*(_p3.y - _p1.y) - (_p2.y - _p1.y)*(_p3.x - _p1.x);
    const float D = -(A*_p1.x + B*_p1.y + C*_p1.z);

    return std::abs(A*_from.x + B*_from.y + C*_from.z + D) / std::sqrt(A*A + B*B + C*C);
}

struct Node;

class KdTree3D
{
public:

    KdTree3D();

    void insert(std::vector<float> point, int id);
    std::vector<int> search(std::vector<float> target, float distanceTol);

private:

    void insert(Node** _node, int _depth, const std::vector<float>& _point, int _id);
    void search(std::vector<int>& _ids, int _depth, Node* _node, const std::vector<float>& _target, float _distance);

    Node* root;
};

void proximity(const std::vector<std::vector<float> >& _points, int _current, std::vector<int>& _cluster,
               std::unordered_set<int>& _processed, KdTree3D* _tree, float _distanceTol);

std::vector<std::vector<int> > euclideanCluster(const std::vector<std::vector<float> >& _points, KdTree3D* _tree, float _distanceTol);

#endif // HELPERS_H_
