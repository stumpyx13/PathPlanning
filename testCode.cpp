#include "std_lib_facilities.h"
#include "environment_classes.h"

#include "tree_classes.h"
#include "tree_classes.cpp"
#include "RRT_classes.h"
#include "RRT_classes.cpp"
int main()
{
	Environment env = Environment(50.0, 50.0);
	auto obs1_p = std::make_shared<Obstacle>(25.0,25.0,5.0,5.0);
	auto obs2_p = std::make_shared<Obstacle>(10,20,5,5);

	env.addObstacle(obs1_p);
	env.addObstacle(obs2_p);

	Point start_point(1.0,1.0);
	Obstacle goal_region(45.0,49.0,2.0,2.0);

	int N_samples = 30000;
	double radius = 1.5;

	RRT_star<Point> rrtObject(N_samples, env, goal_region, start_point);
	std::cout << "Initiating RRT!" << std::endl;
	rrtObject.initiate(radius);
	std::cout << "RRT initiaton/calculation complete!" << std::endl;
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
