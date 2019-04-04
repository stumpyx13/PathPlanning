#include "RRT_classes.h"

template<typename T>
RRT_star<T>::RRT_star(int N, Environment& env_input, Obstacle& goal_in,
		T& start_in){
	N_points = N;
	nodeList.reserve(N_points);
	env = env_input;
	goalRegion = goal_in;
	start = start_in;
}

template<typename T>
void RRT_star<T>::addNode(std::shared_ptr<TreeNode<T>> newNode){
	nodeList.push_back(newNode);
}

// Extend function has distinct steps:
// 1) generate a random item/point
// 2) Attempt to link the generated point to the nearest item/point
// 3) Check if there are nearby nodes that allow a path to the generated n
// node with a lower cost than the nearest, link instead if that is the case
// 4) For the same nearby nodes, see if the nearby nodes can be reached 
// with a lower cost through the new generated node then the original 
// path that leads to them. If so, rewire such that the path flows through
// the generated node (rewiring step)
template<typename T>
bool RRT_star<T>::extend(const double radius){
	bool extend_success = false; //keeps track of whether node if
	//extend leads to a new TreeNode
	//Generate random point/item
	std::shared_ptr<T> p_proposedItem = std::make_shared<T>(T());
	p_proposedItem->genRandom();
	
	// Find nearest node
	std::shared_ptr<TreeNode<T>> nearestNode 
		= getNearestNode(p_proposedItem);

	// Steer towards generated point/item from nearest node
	std::shared_ptr<T> newItem = steer(nearestNode, p_proposedItem);
	
	// Check if new item can be reached through the nearest node
	if(!collisionCheck(newItem, nearestNode->getItem())){
		extend_success = true;
		// make TreeNode object for the generated node
		std::shared_ptr<TreeNode<T>> newNode = 
			std::make_shared<TreeNode<T>>
			(TreeNode<T>(newItem, nearestNode));

		// Determine cost of new TreeNode
		newNode->setCost(nearestNode->getCost() 
				+ calculateCost(newNode));
		double bestCost = newNode.getCost();
		auto p_minNode = nearestNode;
		// Get nearby nodes
		auto p_nearNodes = getNearNodes(newItem, radius);
		for(auto nearNode : p_nearNodes){
		// For each nearby node, check if generated node can be
		// reached through the nearby node
		// If so, calculate cost to do so and see if the path is
		// better than the current path through the nearest node
		// If cost is lower, prefer this path instead
			if(!collisionCheck(newItem, nearNode->getItem())){
				newNode->setParent(nearNode);
				double nearCost = nearNode->getCost() 
					+ calculateCost(newNode);
				if(nearCost < bestCost){
					bestCost = nearCost;
					p_minNode = nearNode;
				}
			}
		}

		// Check if nearby nodes can be reached through the
		// generated node at a lower cost than the cost
		// to reach them currently. If so, rewire to go through
		// generated node
		newNode->setParent(p_minNode);
		newNode->setCost(bestCost);
		for(auto nearNode : p_nearNodes){
			if(nearNode == p_minNode){}
			else{
				auto parentNode_org = nearNode->getParent();
				nearNode->setParent(newNode);
				if(!collisionCheck
					(newItem,nearNode->getItem()) &&
					nearNode->getCost() > 
					newNode->getCost() 
					  + calculateCost(nearNode)){}
				else{
					nearNode->setParent(parentNode_org);
				}		
			}
		}
		addNode(newNode);
	}
	return extend_success;	
}

// collisionCheck iterates through Obstacles in the Environment object used
// to see if the line connecting two items/points results in a collision
// Boolean indicates whether or not collision happens
template<typename T>
bool RRT_star<T>::collisionCheck(std::shared_ptr<T> p1, std::shared_ptr<T> p2){
	bool collision = 0;
	std::shared_ptr<Line> proposedLine = std::make_shared<Line>(Line(p1, p2));
	if(env.obstacleFree(p1) && env.obstacleFree(p2)){
		for(auto ob : env.getObstacleList()){
			if(ob->lineIntersects(proposedLine)){
				collision = 1;
				break;
			}
		}
	}
	else{
		collision = 1;
	}
	return collision;
}


template<typename T>
std::shared_ptr<TreeNode<T>> RRT_star<T>::getNearestNode(
	const std::shared_ptr<T> p_proposedItem) const{
	
	std::shared_ptr<TreeNode<T>> current_nearestNode = nullptr;
	double bestDistance = 1000;
	for (auto p_node_check : nodeList){
		auto p_item_check = p_node_check->getItem();
		auto distance = 
			p_proposedItem->calculateDistance(p_item_check);
		if(distance < bestDistance || 
				current_nearestNode ==nullptr){
			current_nearestNode = p_node_check;
		}
	}
	return current_nearestNode;
}

template<typename T>
std::shared_ptr<T> RRT_star<T>::steer(const std::shared_ptr<TreeNode<T>> p_nearestNode, const std::shared_ptr<T> p_proposedItem){
	auto nearestItem = p_nearestNode->getItem();
	double max_distance = 0.5; // Currently hard-coded for testing, need to change
	return nearestItem->moveTowards(p_proposedItem, max_distance);
		
}

template<typename T>
std::vector<std::shared_ptr<TreeNode<T>>> RRT_star<T>::getNearNodes(const std::shared_ptr<T> p_node, const double radius) const{
	std::vector<std::shared_ptr<TreeNode<T>>> nearNodeList;
	for (auto p_node_check : nodeList){
		auto p_item = p_node_check->getItem();
		auto distance = p_item->calculateDistance(p_node);
		if(distance <= radius){
			nearNodeList.push_back(p_node);
		}
	}
	return nearNodeList;
}

template<typename T>
double RRT_star<T>::calculateCost(const std::shared_ptr<TreeNode<T>> node) const{
	std::shared_ptr<T> item_cur = node->getItem();
        std::shared_ptr<TreeNode<T>> parent = node->getParent();
	std::shared_ptr<T> item_par = parent->getItem();	
	return item_cur->calculateDistance(item_par);	
}
