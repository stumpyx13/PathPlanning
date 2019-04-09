#include "environment_classes.h"
#include <cmath>
#include <random>

Environment::Environment(double setdeltaX, double setdeltaY){
	deltaX = setdeltaX;
	deltaY = setdeltaY;
}

double Environment::getDeltaX() const{
	return deltaX;
}
double Environment::getDeltaY() const{
	return deltaY;
}

// functions
void Environment::addObstacle(std::shared_ptr<Obstacle> obs){// Add an obstacle to the environement
	obstacleList.push_back(obs);
}

std::vector<std::shared_ptr<Obstacle>> Environment::getObstacleList() const{// return a vector of obstacles that exist in the environment
	return obstacleList;
}
bool Environment::obstacleFree(std::shared_ptr<Point> p) const{// check if a point does not fall in an obstacle in the environment
	bool indicator = true;
	if(obstacleList.empty()){//if no obstacles, point is obstacle free automatically
		return indicator;
	}	
	else{ 
		for(const auto it: obstacleList){//iterate through obstacles and check if point is in each obstacle
			indicator  = it->inObstacle(p);
			if(indicator){//if point is in an obstacle, return false immediately, no need to iterate further
				return false;
			}
		}
	//if for loop completes successfully, no obstacles are hit
		return true;
	}
}

void Environment::printItem(std::ofstream& os) const{
	os << deltaX << "," << deltaY << "\n";
}

double Environment::getMaxDistance() const{
	return std::sqrt(pow(deltaX,2)+pow(deltaY,2));
}

void Environment::generateRandomObstacles(const int N_obstacles,
		const double sizeBound){
	for(int i = 1; i<= N_obstacles; i++){
		std::shared_ptr<Obstacle> newOb =
			std::make_shared<Obstacle>();
		newOb->genRandom(deltaX, deltaY, sizeBound);
		obstacleList.push_back(newOb);
	}
}

Obstacle::Obstacle(const double xMax, const double yMax, const double sizeBound){
	genRandom(xMax, yMax, sizeBound);
}
	
Obstacle::Obstacle(double xSet, double ySet, double dxSet, double dySet){
	x = xSet;
	y = ySet;
	dx = dxSet;
	dy = dySet;

	pointList[0] = std::make_shared<Point>(Point{x,y});
	pointList[1] = std::make_shared<Point>(Point{x+dx,y});
	pointList[2] = std::make_shared<Point>(Point{x,y-dy});
	pointList[3] = std::make_shared<Point>(Point{x+dx,y-dy});

	lineList[0] = std::make_shared<Line>(Line(pointList[0],pointList[1]));
	lineList[1] = std::make_shared<Line>(Line(pointList[0],pointList[2]));
	lineList[2] = std::make_shared<Line>(Line(pointList[1],pointList[3]));
	lineList[3] = std::make_shared<Line>(Line(pointList[2],pointList[3]));
}

bool Obstacle::inObstacle(std::shared_ptr<Point> p) const{// Check if point is in the obstacle
	// get point coordinates
	double p_x = p->getX();
	double p_y = p->getY();
	// check if point is within obstacle boundaries
	bool xCheck = (p_x >= x) && (p_x <= x+dx);
	bool yCheck = (p_y <= y) && (p_y >= y-dy);
	return (xCheck && yCheck);	
}

bool Obstacle::lineIntersects(const std::shared_ptr<Line> p_line) const{
	bool intersect = false;	
	for (auto p_line_check : lineList){
		if(p_line->intersectCheck(p_line_check)){
			intersect = true;
			break;
		}
	}
	return intersect;
}

double Obstacle::getdx() const{
	return dx;
}
double Obstacle::getdy() const{
	return dy;
}
double Obstacle::getX() const{
	return x;
}
double Obstacle::getY() const{
	return y;
}

void Obstacle::printItem(std::ofstream& os) const{
	os << x << "," << y << "," << dx << "," << dy << "\n";
}

void Obstacle::genRandom(const double xMax, const double yMax,const double sizeBound){
	std::random_device rd;
	std::mt19937 gen(rd());
	double lower_bound = 0;
	std::uniform_real_distribution<> xDistribution(lower_bound,xMax);
	std::uniform_real_distribution<> yDistribution(lower_bound,yMax);
	std::uniform_real_distribution<> sizeDistribution(lower_bound,sizeBound);
	x = xDistribution(gen);
	y = yDistribution(gen);
	dx = sizeDistribution(gen);
	dy = sizeDistribution(gen);
	pointList[0] = std::make_shared<Point>(Point{x,y});
	pointList[1] = std::make_shared<Point>(Point{x+dx,y}); 	
	pointList[2] = std::make_shared<Point>(Point{x,y-dy});
	pointList[3] = std::make_shared<Point>(Point{x+dx,y-dy});
	lineList[0] = std::make_shared<Line>(Line(pointList[0],pointList[1]));
	lineList[1] = std::make_shared<Line>(Line(pointList[0],pointList[2])); 
	lineList[2] = std::make_shared<Line>(Line(pointList[1],pointList[3])); 
	lineList[3] = std::make_shared<Line>(Line(pointList[2],pointList[3]));
}
Point::Point(){}
                                                                       
Point::Point(double xSet, double ySet){
	x = xSet;
	y = ySet;
}
double Point::getX() const{
	return x;
}
double Point::getY() const{
	return y;
}
void Point::setX(double xSet){
	x = xSet;
}
void Point::setY(double ySet){
	y = ySet;
}
void Point::printItem(std::ofstream& os) const{
	os << x << "," << y <<"\n";
}
double Point::calculateDistance(const std::shared_ptr<Point> p) const{
	double p_x = p->getX();
	double p_y = p->getY();
	return std::sqrt(pow(p_x-x,2) + pow(p_y-y,2));
}

void Point::genRandom(const Environment& env){
	std::random_device rd; //Get random seed
	std::mt19937 gen(rd()); //mersenne_twister_engine seeded with rd()
	double lower_bound = 0;
	std::uniform_real_distribution<> xDistribution(lower_bound, env.getDeltaX());
	std::uniform_real_distribution<> yDistribution(lower_bound, env.getDeltaY());

	x = xDistribution(gen);
	y = yDistribution(gen);
}

std::shared_ptr<Point> Point::moveTowards(const std::shared_ptr<Point> p_goal,const double dist) const{
	double goal_dist = calculateDistance(p_goal);
	if (goal_dist <= dist){
		return p_goal;
	}
	else{
		double proportion_dist = dist/goal_dist;
		double goalX = p_goal->getX();
		double goalY = p_goal->getY();
	       	return std::make_shared<Point>(
			(proportion_dist*(goalX-x))+x,
			(proportion_dist*(goalY-y))+y);
	}
}

Line::Line(){};

Line::Line(std::shared_ptr<Point> p1_in, std::shared_ptr<Point> p2_in){
	p1 = p1_in;
	p2 = p2_in;
}

std::shared_ptr<Point> Line::getPoint1() const{
	return p1;
}
std::shared_ptr<Point> Line::getPoint2() const{
	return p2;
}

void Line::setPoint1(std::shared_ptr<Point> p1_in){
	p1 = p1_in;
}
void Line::setPoint2(std::shared_ptr<Point> p2_in){
	p2 = p2_in;
}
void Line::setPoints(std::shared_ptr<Point> p1_in, std::shared_ptr<Point> p2_in){
	p1 = p1_in;
	p2 = p2_in;
}
bool Line::intersectCheck(std::shared_ptr<Line> line_check) const{
	bool intersect = false;
	double x1 = p1->getX();
	double x2 = p2->getX();
	double y1 = p1->getY();
	double y2 = p2->getY();
	
	std::shared_ptr<Point> p3 = line_check->getPoint1();
	std::shared_ptr<Point> p4 = line_check->getPoint2();
	double x3 = p3->getX();
	double x4 = p4->getX();
	double y3 = p3->getY();
	double y4 = p4->getY();
	
	double check1 = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) /
		((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
	double check2 = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) /
		((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
	
	if(check1 >= 0 && check1 <= 1 && check2 >= 0 && check2 <= 1){
		intersect = true;	
	}
	return intersect;	
}
