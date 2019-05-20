#ifndef RRT_H_INCLUDED
#define RRT_H_INCLUDED
#include "std_lib_facilities.h"
#include "tree_classes.h"
#include "tree_classes.cpp"
#include "environment_classes.h"

// RRT* algorithm, formulated as a template class definition
// RRT* constructor initializes algorithm calculation
// Finds best sampled path from start point to goal region; goal region is formualted as an instance of the Obstacle class (see environment_classes.h or .cpp)
// Naming convention of member functions follow the naming convention of the original RRT* paper (Karaman; Frizzoli 2010), see paper for general goal of each member function
// TreeNodes, points, lines, and obstacles are generally placed on the heap, with smart pointers used to access without utilizing new/delete
//
// Requirements of the template T type/class:
// 1) has a calculateDistance(std::shared_ptr<T> p_item) member function
// that returns the distance between the item and the item that the input
// pointer points to
// 2) Can be tested to see if it falls in an Obstacle
// 3) has a genRandom() member function that generates random parameters
// 4) A Line can connect two instances of T
//
// Log:
// 3/20/19: Initial creation
template<typename T>
class RRT_star{
	protected:
		int N_points = 1; // Number of points to sample successfully
		std::vector<std::shared_ptr<TreeNode<T>>> nodeList;
		std::shared_ptr<Environment> env; // Environment to apply RRT* algorithm
		std::shared_ptr<Obstacle> goalRegion; // Goal region
		std::shared_ptr<T> start; // Start/root of the tree

		std::vector<std::shared_ptr<TreeNode<T>>> goalNodes;

		// Internal member functions
		bool extend(const double radius); // return value indicates whether or not extend succeeded (obstacle free, new node added)
		bool collisionCheck(std::shared_ptr<T> p1,
				std::shared_ptr<T> p2); // indicates whether or not the path between p1 and p2 has a collision (0 = no collision)

		std::shared_ptr<TreeNode<T>> getNearestNodeParallel(
			const std::shared_ptr<T> p_proposedItem) const;

		std::shared_ptr<TreeNode<T>> getNearestNode_single(
			const std::shared_ptr<T> p_proposedItem) const;

		std::pair<double,std::shared_ptr<TreeNode<T>>> getNearestNode_worker(
				const std::shared_ptr<T> p_proposedItem,
				const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator start_i,
				const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator end_i) const;

		std::vector<std::shared_ptr<TreeNode<T>>> getNearNodesParallel(
			const std::shared_ptr<T> p_node, const double radius);

		std::vector<std::shared_ptr<TreeNode<T>>>
			getNearNodes_single(const std::shared_ptr<T> p_node,
				     const double radius) const; // returns all nodes within a set radius of the node pointed to by p_node

	 std::vector<std::shared_ptr<TreeNode<T>>> getNearNodes_worker(const std::shared_ptr<T> p_node,
			const double radius, const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator start_it,
			const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator end_it) const;

	 double calculateCost(const std::shared_ptr<TreeNode<T>>
				node) const; // calculate the cost betweensome node and its parent

	 std::shared_ptr<T> steer(const
			std::shared_ptr<TreeNode<T>> p_nearestNode,
			const std::shared_ptr<T> p_proposedItem); // steers the nearest node item towards the proposed item, dictated by chosen dynamics

	public:
		// constructor, initializes algorithm
		RRT_star(int N, Environment& env_input, Obstacle& goal_in,
				T& start_in);

		// Public Member functions
		void addNode(std::shared_ptr<TreeNode<T>> newNode);

		void initiate(double radius); //initiate RRT calculation
		TreeAncestorPath<T> getFinalPath(); // get path resulting from RRT* calculation, if multiple paths exist, choose the best one
		void printNodes(std::ofstream& os) const;
};
#endif
