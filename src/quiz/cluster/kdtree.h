/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

bool box2DContains(const std::vector<float>& _center,
                 float _side,
                 const std::vector<float>& _point)
{
    return _center.front() - _side <= _point.front() && _center.front() + _side >= _point.front() &&
            _center.back() - _side <= _point.back() && _center.back() + _side >= _point.back();
}

float euclideanDistance2D(const std::vector<float>& _first, const std::vector<float>& _second)
{
    return std::sqrt(std::pow(_first.front() - _second.front(), 2) + std::pow(_first.back() - _second.back(), 2));
}

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

        if (!root)
        {
            root = new Node(point, id);
        }
        else
        {
            insert(&root, 0, point, id);
        }
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

        search(ids, 0, root, target, distanceTol);

		return ids;
	}

private:
    void insert(Node** _node, int _depth, const std::vector<float>& _point, int _id)
    {
        if (!(*_node))
        {
            *_node = new Node(_point, _id);
        }
        else
        {
            _depth += 1;

            const bool compareX = (bool)(_depth % 2);

            if (compareX)
            {
                if (_point.front() < (*_node)->point.front())
                {
                    insert(&((*_node)->left), _depth, _point, _id);
                }
                else
                {
                    insert(&((*_node)->right), _depth, _point, _id);
                }
            }
            else
            {
                if (_point.back() < (*_node)->point.back())
                {
                    insert(&((*_node)->left), _depth, _point, _id);
                }
                else
                {
                    insert(&((*_node)->right), _depth, _point, _id);
                }
            }
        }
    }

    void search(std::vector<int>& _ids, int _depth, Node* _node, const std::vector<float>& _target, float _distance)
    {
        if (!_node)
        {
            return;
        }
        else
        {
            _depth += 1;

            if (box2DContains(_target, _distance, _node->point))
            {
                if (euclideanDistance2D(_target, _node->point) <= _distance)
                {
                    _ids.push_back(_node->id);
                }

                search(_ids, _depth, _node->left, _target, _distance);
                search(_ids, _depth, _node->right, _target, _distance);
            }
            else
            {
                const bool compareX = (bool)(_depth % 2);

                if (compareX)
                {
                    if (_target.front() - _distance <= _node->point.front() && _target.front() + _distance >= _node->point.front())
                    {
                        search(_ids, _depth, _node->left, _target, _distance);
                        search(_ids, _depth, _node->right, _target, _distance);
                    }
                    else if (_target.front() + _distance < _node->point.front())
                    {
                        search(_ids, _depth, _node->left, _target, _distance);
                    }
                    else
                    {
                        search(_ids, _depth, _node->right, _target, _distance);
                    }
                }
                else
                {
                    if (_target.back() - _distance <= _node->point.back() && _target.back() + _distance >= _node->point.back())
                    {
                        search(_ids, _depth, _node->left, _target, _distance);
                        search(_ids, _depth, _node->right, _target, _distance);
                    }
                    else if (_target.back() + _distance < _node->point.back())
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
    }
};
