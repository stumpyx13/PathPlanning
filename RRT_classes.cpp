#include "RRT_classes.h"

template<typename T>
RRT_star<T>::RRT_star(int N, Environment& env_input, Obstacle& goal_in,
		T& start_in){
	N_points = N;
	nodeList.reserve(N_points);
	env = std::make_shared<Environment>(env_input);
	goalRegion = std::make_shared<Obstacle>(goal_in);
	start = std::make_shared<T>(start_in);
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
	p_proposedItem->genRandom(*env);
	//std::cout << "Point generated: (" << p_proposedItem->getX() << ","<< p_proposedItem->getY() << ")" << std::endl;
	// Find nearest node
	std::shared_ptr<TreeNode<T>> nearestNode
		= getNearestNodeParallel(p_proposedItem);

	// Steer towards generated point/item from nearest node
	std::shared_ptr<T> newItem = steer(nearestNode, p_proposedItem);
	//std::cout << "Point steered to: (" << newItem->getX() << "," << newItem->getY() <<")" << std::endl;
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
		double bestCost = newNode->getCost();
		auto p_minNode = nearestNode;
		// Get nearby nodes
		auto p_nearNodes = getNearNodesParallel(newItem, radius);
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
			if(nearNode != p_minNode){
				auto parentNode_org = nearNode->getParent();
				nearNode->setParent(newNode);
				if(!collisionCheck
					(newItem,nearNode->getItem()) &&
					nearNode->getCost() >
					newNode->getCost()
					  + calculateCost(nearNode)){

					nearNode->setCost(newNode->getCost() + calculateCost(nearNode));
					nearNode->setParent(newNode);

				}
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
	bool collision = false;
	if(env->obstacleFree(p1) && env->obstacleFree(p2)){
		std::shared_ptr<Line> proposedLine = std::make_shared<Line>(p1, p2);
		for(auto ob : env->getObstacleList()){
			if(ob->lineIntersects(proposedLine)){
				collision = true;
				break;
			}
		}
	}
	else{
		collision = true;
	}
	return collision;
}


template<typename T>
std::shared_ptr<TreeNode<T>> RRT_star<T>::getNearestNodeParallel(
	const std::shared_ptr<T> p_proposedItem) const{

	if(nodeList.size() < 100000){return getNearestNode_single(p_proposedItem);}

	int N_tasks = 8;
	std::pair<double,std::shared_ptr<TreeNode<T>>> worker_best =
		std::make_pair<double,std::shared_ptr<TreeNode<T>>>(0,nullptr);
	int N_splitNodes = nodeList.size()/N_tasks;
	std::vector<std::future<std::pair<double,std::shared_ptr<TreeNode<T>>>>> threaded_futures;

	for(int i = 0; i<N_tasks;++i){ //starting threads
		auto start_it = nodeList.cbegin()+ i * (N_splitNodes);
		auto end_it = i == N_tasks-1 ? nodeList.cend() : nodeList.cbegin() + (i+1) * N_splitNodes;
		// bind with worker function with lambda function
		auto task = [this](const std::shared_ptr<T> p,
		const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator start_i,
		const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator end_i){
				return getNearestNode_worker(p,start_i,end_i);};

		// start async task
		threaded_futures.emplace_back(async(task,p_proposedItem,start_it,end_it));
	}

	for(int i = 0; i < N_tasks; ++i){
		std::pair<double,std::shared_ptr<TreeNode<T>>> local_best = threaded_futures[i].get();
		if(local_best.first > worker_best.first){worker_best = local_best;}
	}

	return worker_best.second;
}

template<typename T>
std::shared_ptr<TreeNode<T>> RRT_star<T>::getNearestNode_single(
	const std::shared_ptr<T> p_proposedItem) const{

	std::shared_ptr<TreeNode<T>> current_nearestNode = nullptr;
	double bestDistance = env->getMaxDistance();
	bestDistance *= bestDistance; // Used to accomodate Point::calculateCost usage instead of calculateDistance
	for (auto p_node_check : nodeList){
		auto p_item_check = p_node_check->getItem();
		auto distance =
			p_proposedItem->calculateCost(p_item_check); // Point::calculateCost is distance without the sqrt
		if(distance < bestDistance ||
				current_nearestNode ==nullptr){
			current_nearestNode = p_node_check;
			bestDistance = distance;
		}
	}
	return current_nearestNode;
}

template<typename T>
std::pair<double,std::shared_ptr<TreeNode<T>>> RRT_star<T>::getNearestNode_worker(
	const std::shared_ptr<T> p_proposedItem,
	const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator start_i,
	const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator end_i) const{

	std::shared_ptr<TreeNode<T>> current_nearestNode = nullptr;
	double bestDistance = env->getMaxDistance();
	bestDistance *= bestDistance; // Used to accomodate Point::calculateCost usage instead of calculateDistance
	for (auto it = start_i; it < end_i; ++it){
		auto p_node_check = *it;
		auto p_item_check = p_node_check->getItem();
		auto distance =
			p_proposedItem->calculateCost(p_item_check); // Point::calculateCost is distance without the sqrt
		if(distance < bestDistance ||
				current_nearestNode ==nullptr){
			current_nearestNode = p_node_check;
			bestDistance = distance;
		}
	}
	return std::make_pair(bestDistance,current_nearestNode);
}

template<typename T>
std::shared_ptr<T> RRT_star<T>::steer(const std::shared_ptr<TreeNode<T>> p_nearestNode, const std::shared_ptr<T> p_proposedItem){
	auto nearestItem = p_nearestNode->getItem();
	double max_distance = 0.5; // Currently hard-coded for testing, need to change
	return nearestItem->moveTowards(p_proposedItem, max_distance);

}

template<typename T>
std::vector<std::shared_ptr<TreeNode<T>>> RRT_star<T>::getNearNodesParallel(
	const std::shared_ptr<T> p_node, const double radius){
		int N_threads = 8;
		if(nodeList.size() > N_threads*10000){
			int N_splitNodes = nodeList.size()/N_threads;
			using Task_type = std::vector<std::shared_ptr<TreeNode<T>>>(std::shared_ptr<T>,
			double, std::vector<std::shared_ptr<TreeNode<T>>>);

			std::vector<std::packaged_task<Task_type>> threaded_tasks;
			std::vector<std::future<std::vector<std::shared_ptr<TreeNode<T>>>>> threaded_futures;

			// // Packaging tasks
			// for(int i = 0; i<N_threads; ++i){
			// 	//create packaged_task via. lambda function
			// 	std::packaged_task<Task_type> pt([this](std::shared_ptr<T> p, double r,
			// 	const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator start_i,
			// 	const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator end_i){
			// 		return getNearNodes_worker(p,r,start_i,end_i);});
			//
			// 	assert(pt.valid());
			//
			// 	threaded_futures.emplace_back(pt.get_future());
			// 	threaded_tasks.push_back(std::move(pt));
			//
			// }
			//
			//start threads
			//std::vector<std::thread> thread_workers;
			for(int i = 0; i<N_threads;++i){ //starting threads
				auto start_it = nodeList.cbegin()+ i * (N_splitNodes);
				auto end_it = i == N_threads-1 ? nodeList.cend() : nodeList.cbegin() + (i+1) * N_splitNodes;
				// bind with worker function with lambda function
				auto task = [this](std::shared_ptr<T> p, double r,
				 	const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator start_i,
				 	const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator end_i){
				 		return getNearNodes_worker(p,r,start_i,end_i);};
				threaded_futures.emplace_back(async(task,p_node,
				radius,start_it,end_it));
				// thread_workers.emplace_back(std::move(threaded_tasks[i]),
				// p_node,radius,start_it, end_it);
			}

			// concatenate vector results
			std::vector<std::shared_ptr<TreeNode<T>>> returnVect;

			for(int i = 0; i<N_threads; ++i){
				//thread_workers[i].join();
				std::vector<std::shared_ptr<TreeNode<T>>> fut = threaded_futures[i].get();
				returnVect.insert(returnVect.end(),fut.begin(),fut.end());
			}
			return returnVect;
		}
		else{return getNearNodes_single(p_node,radius);}
	}

template<typename T>
std::vector<std::shared_ptr<TreeNode<T>>> RRT_star<T>::getNearNodes_worker(const std::shared_ptr<T> p_node,
	const double radius, const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator startIt,
	const typename std::vector<std::shared_ptr<TreeNode<T>>>::const_iterator endIt) const{

	std::vector<std::shared_ptr<TreeNode<T>>> nearNodeList;

	for (auto it = startIt; it < endIt; ++it){
		std::shared_ptr<TreeNode<T>> node_check = *it;
		auto p_item = node_check->getItem();
		auto distance = p_item->calculateCost(p_node); // Point::calculateCost is distance without the sqrt
		double r = radius*radius;
		if(distance <= r){
			nearNodeList.push_back(node_check);
		}
	}
	return nearNodeList;
}

template<typename T>
std::vector<std::shared_ptr<TreeNode<T>>> RRT_star<T>::getNearNodes_single(const std::shared_ptr<T> p_node, const double radius) const{
	std::vector<std::shared_ptr<TreeNode<T>>> nearNodeList;

	for (auto p_node_check : nodeList){
		auto p_item = p_node_check->getItem();
		auto distance = p_item->calculateCost(p_node); // Point::calculateCost is distance without the sqrt
		double r = radius*radius;
		if(distance <= r){
			nearNodeList.push_back(p_node_check);
		}
	}
	return nearNodeList;
}

template<typename T>
double RRT_star<T>::calculateCost(const std::shared_ptr<TreeNode<T>> node) const{
	std::shared_ptr<T> item_cur = node->getItem();
        std::shared_ptr<TreeNode<T>> parent = node->getParent();
	std::shared_ptr<T> item_par = parent->getItem();
	return item_cur->calculateCost(item_par);
}

template<typename T>
void RRT_star<T>::initiate(double radius){
	// Required assumptions:
	// 1) goalRegion sits in the environment env
	// 2) goalRegion is a region (not a single point)
	// 3) Environment, goal region, and start point exist

	// check assumptions
	bool goalRegion_x_greaterThanZero = goalRegion->getdx() > 0;
	bool goalRegion_y_greaterThanZero = goalRegion->getdy() > 0;
	assert(goalRegion_x_greaterThanZero);
	assert(goalRegion_y_greaterThanZero);

	bool env_x_greaterThanZero = env->getDeltaX() > 0;
	bool env_y_greaterThanZero = env->getDeltaY() > 0;

	assert(env_x_greaterThanZero);
	assert(env_y_greaterThanZero);

	bool goalRegion_x_inEnvironment = goalRegion->getX() < env->getDeltaX();
	bool goalRegion_y_inEnvironment = goalRegion->getY() < env->getDeltaY();
	assert(goalRegion_x_inEnvironment);
	assert(goalRegion_y_inEnvironment);

	// main routine
	std::shared_ptr<TreeNode<T>> startNode =
		std::make_shared<TreeNode<T>>(TreeNode<T>(start)); //make startNode
	nodeList.push_back(startNode); //add start node
	for(int i = 1; i < N_points; i++){
		bool extendSuccess =  extend(radius);
		if(extendSuccess){
			//std::cout << "RRT node added at iteration: " << i << std::endl;
			auto lastItemAdded = nodeList.back()->getItem();
			if(goalRegion->inObstacle(lastItemAdded)){
				goalNodes.push_back(nodeList.back());
			//	std::cout << "Goal reached at iteration: "<< i << std::endl;
			}
		}
		else{
			//std::cout << "Extend failed at iteration: " << i << std::endl;
		}

	}
}

template<typename T>
TreeAncestorPath<T> RRT_star<T>::getFinalPath(){
	double currentBestCost = -1;
	std::shared_ptr<TreeNode<T>> bestGoalNode;
	assert(!goalNodes.empty());
	for(auto p_node_check : goalNodes){
		if(currentBestCost == -1 ||
				p_node_check->getCost() < currentBestCost){
			bestGoalNode = p_node_check;
			currentBestCost = p_node_check->getCost();
		}
	}
	return TreeAncestorPath<T>(*bestGoalNode);
}

template<typename T>
void RRT_star<T>::printNodes(std::ofstream& os) const{
	for (auto p_current_node : nodeList){
		auto p_current_item = p_current_node->getItem();
		p_current_item->printItem(os);
	}
}
