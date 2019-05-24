#include "std_lib_facilities.h"
#include "environment_classes.h"

#include "tree_classes.h"
#include "tree_classes.cpp"
#include "RRT_classes.h"
#include "RRT_classes.cpp"
#include <chrono>
int main()
{
	Environment env = Environment(50.0, 50.0);
	auto obs1_p = std::make_shared<Obstacle>(10.0,50.0,2.0,45.0);
	auto obs2_p = std::make_shared<Obstacle>(20,40,2,45);
	auto obs3_p = std::make_shared<Obstacle>(30,50,2,45);

	env.addObstacle(obs1_p);
	env.addObstacle(obs2_p);
	env.addObstacle(obs3_p);
	//env.generateRandomObstacles(10,8.0);
	Point start_point(1.0,48.0);
	Obstacle goal_region(43.0,49.0,7.0,2.0);
	auto obs_list = env.getObstacleList();
	int N_samples;
	double radius = 4;
	std::cout << "Number of samples?: ";
	std::cin >> N_samples;
	std::cout << std::endl;
	RRT_star<Point> rrtObject(N_samples, env, goal_region, start_point);
	std::cout << "Initiating RRT!" << std::endl;
	auto start = std::chrono::high_resolution_clock::now();
	rrtObject.initiate(radius);
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
	std::cout << "RRT initiaton/calculation complete" << std::endl;
	std::cout << "Calculation time: " << double(duration.count())/1000000.0 << " seconds"<<std::endl;
	TreeAncestorPath<Point> finalPath = rrtObject.getFinalPath();
	std::cout << "Final path retrieved!" << std::endl;
//	Point p_check1;
//	p_check1.genRandom(env);
//	Point p_check2;
//	p_check2.genRandom(env);

	std::shared_ptr<Point> pointer_p1_check = std::make_shared<Point>(31,23);

//	bool obsCheck1 = obs1_p->inObstacle(pointer_p1_check);
	bool obsCheck2 = env.obstacleFree(pointer_p1_check);

//	std::cout << "Point 1: (" << p_check1.getX() << "," <<
//		p_check1.getY() << ")" << std::endl;
//	std::cout << "Point 2: (" << p_check2.getX() << "," <<
//		p_check2.getY() << ")" << std::endl;
//	std::cout << "Obstacle check 1: "<< obsCheck1 << std::endl;
	std::cout << "Obstacle check 1 inverse: " << obsCheck2 << std::endl;
//	std::cout << "Distance between points: " << p_check1.calculateDistance(std::make_shared<Point>(p_check2)) << std::endl;
//	TreeNode<Point> node1(std::make_shared<Point>(p_check1));
//	TreeNode<Point> node2(std::make_shared<Point>(p_check2), std::make_shared<TreeNode<Point>>(node1));
//	TreeAncestorPath<Point> treePath(node2);
	std::ofstream myfile ("optimalPath_RRTstar.csv");
	if(myfile.is_open()){
		std::cout << "Printing final path!" << std::endl;
		finalPath.printPath(myfile);
		myfile.close();
	}
	else std::cout << "Unable to open file" << std::endl;

	std::ofstream nodeFile("nodesSampled_RRTstar.csv");
	if(nodeFile.is_open()){
		std::cout << "Printing node list!" << std::endl;
		rrtObject.printNodes(nodeFile);
		nodeFile.close();
	}
	else std::cout << "Unable to open node file" << std::endl;


	std::ofstream envFile("environment.csv");
	if(envFile.is_open()){
		std::cout << "Printing environment parameters!" << std::endl;
		env.printItem(envFile);
		start_point.printItem(envFile);
		goal_region.printItem(envFile);
		envFile.close();
	}

	std::ofstream obsFile("obstacles.csv");
	if(obsFile.is_open()){
		std::cout << "Printing obstacles!" << std::endl;
		for (auto ob : env.getObstacleList()){
			ob->printItem(obsFile);
		}
		obsFile.close();
	}

	return 0;
}
