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

// Obstacle =================================================================

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

// Point =====================================================================
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

double Point::calculateCost(const std::shared_ptr<Point> p) const{
	// used instead of calculateDistance to reduce computation time (actual distance doesn't matter for comparison
	// saves sqrt calculation time
	double dx = p->getX()-x;
	double dy = p->getY()-y;
	return dx*dx + dy*dy;
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

// Dynamic Point ==============================================================

DynamicPoint::DynamicPoint(){};

DynamicPoint::DynamicPoint(double xSet, double ySet, double dxdtSet, double dydtSet,
 double d2xdt2_in, double d2ydt2_in){
	x = xSet; y = ySet; dxdt = dxdtSet; dydt = dydtSet;
}

double DynamicPoint::get_dxdt() const{return dxdt;}
double DynamicPoint::get_dydt() const{return dydt;}
double DynamicPoint::get_dxdtMax() const{return dxdtMax;}
double DynamicPoint::get_dydtMax() const{return dydtMax;}

void DynamicPoint::set_dxdt(double dxdt_in){dxdt = dxdt_in;}
void DynamicPoint::set_dydt(double dydt_in){dydt = dydt_in;}
void DynamicPoint::set_dxdtMax(double dxdtMax_in){dxdtMax = dxdtMax_in;}
void DynamicPoint::set_dydtMax(double dydtMax_in){dydtMax = dydtMax_in;}

// Appropriate distance metric required
// Main issue: "natural magnitude" of position and velocity may be different
// If standard euclidean distance is used in R^4 (for x,y,dy,dx), position
// will be "weighed" more for any distance comparison
// In many cases, we want to avoid this bias towards position

// Currently (5/20/19), distance is quantified (approximated) as minimum time required to
// reach point p from current object point
// calculated as time to cover position difference at max velocity summed with time to
// cover velocity difference at max acceleration
// x and y positions can be changed simultaneously while velocity cannot be changed
// simultaneously
double DynamicPoint::calculateDistance(const std::shared_ptr<DynamicPoint> p) const{
	double p_x = p->getX();
	double p_y = p->getY();
	double p_dxdt = p->get_dxdt();
	double p_dydt = p->getdydt();

	double dx = p_x > x ? p_x-x : x-p_x;
	double dy = p_y > y ? p_y-y : y-p_y;

	double v_diff_x = p_dxdt > dxdt ? p_dxdt-dxdt : dxdt-p_dxdt;
	double v_diff_y = p_dydt > dydt ? p_dydt-dydt : dydt-p_dydt;

	return max(dx/dxdtMax, dy/dydtMax) + (v_diff_x+v_diff_y)/accel_max;
}

double DynamicPoint::calculateCost(const std::shared_ptr<DynamicPoint> p) const{
	// cost parameterized as distance between points as of 5/20/19

	return calculatedDistance(p);
}

// Pseudo-random sequence used for sampling scheme
// Mersenne twister used as sequence generator
// Uniform distribution in the x,y,dxdt,dydt C-space
void DynamicPoint::genRandom(const Environment& env){
	std::random_device rd; //Get random seed
	std::mt19937 gen(rd()); //mersenne_twister_engine seeded with rd()
	double lower_bound_pos = 0;
	std::uniform_real_distribution<> xDistribution(lower_bound_pos, env.getDeltaX());
	std::uniform_real_distribution<> yDistribution(lower_bound_pos, env.getDeltaY());

	std::uniform_real_distribution<> vxDistribution(-dxdtMax,dxdtMax);
	std::uniform_real_distribution<> vyDistribution(-dydtMax,dydtMax);

	x = xDistribution(gen);
	y = yDistribution(gen);
	dxdt = vxDistribution(gen);
	dydt = vyDistribution(gen);
}

// moveTowards characterizes how the tree grows from one point to another
// essentially want to reduce the distance from point to point p_goal
// Given some constant acceleration a, v = v_0 + a*dt (1st order)
// d = d_0 + v_0*dt + 0.5*a*dt^2 (1st order)
// Assume that x and y directions can be handled separately
// Main issue is how to set acceleration vector
// matching the velocity and position vectors of p_goal can be competing
// (imagine going from (0,0,0,0) to (1,1,-1,-1); going from origin to 1,1
// requires velocity vector to be in direction of 1,1, which is opposite of goal
// velocity vector)
// As of 5/21/19, a greedy approach is taken to minimize the distance between the
// object point and p_goal
// Greedy approach taken:
// if acceleration mag to reach spatial position is greater than acceleration
// required to reach velocity of end point, then acceleration is in direction
// of spatial vector from object to goal
// Otherwise, acceleration is in direction to drive current velocity to
// goal velocity
std::shared_ptr<DynamicPoint> DynamicPoint::moveTowards(const std::shared_ptr<DynamicPoint> p_goal,
	const double dt) const{
	double p_x = p_goal->getX(), p_y = p_goal->getY(), p_dxdt = p_goal->get_dxdt(),
		p_dydt = p_goal->getdydt();

	double d_mag = std::sqrt(calculateCost(p_goal));

	double delta_x = p_x - x;
	double delta_y = p_y - y;

	double delta_v_x = p_dxdt - dxdt;
	double delta_v_y = p_dydt - dydt;

	double a_x_req = (delta_x - dxdt * dt)/(0.5*dt*dt);
	double a_y_req = (delta_y - dydt*dt)/(0.5*dt*dt);
	double a_spatial_req = a_x_req*a_x_req + a_y_req*a_y_req;

	double a_x_req_v = delta_v_x/dt;
	double a_y_req_v = delta_v_y/dt;
	double a_vel_req = a_x_req_v*a_x_req_v + a_y_req_v*a_y_req_v;

	// 4 options at this point with greedy approach
	// 1) with accel max, p_goal cannot be reached, so get as far as possible
	// 2) with accel max, p_goal can be reached, so get there then try to match velocity
	// 3) with accel max, velocity cannot be matched, so get as close as possible
	// 4) with accel_max, velocity can be matched, so then try to match distance
	double a_x = 0, a_y = 0;
	if(a_spatial_req > a_vel_req){
		if(a_spatial_req > accel_max){
			a_x = accel_max * a_x_req/a_spatial_req;
			a_y = accel_max * a_y_req/a_spatial_req;
		}
		else{a_x = a_x_req; a_y = a_y_req;}
	}
	else{
		if(a_vel_req > accel_max){
			a_x = accel_max * a_x_req_v/a_vel_req;
			a_y = accel_max * a_y_req_v/a_vel_req;
		}
		else{a_x = a_x_req_v; a_y = a_y_req_v;}
	}
	double x_new = x + dxdt*dt + 0.5 * a_x * dt * dt;
	double y_new = x + dxdt*dt + 0.5 * a_y * dt * dt;
	double dxdt_new = dxdt + a_x*dt;
	double dydt_new = dydt + a_y*dt;
	return std::make_shared<DynamicPoint>(x_new,y_new,dxdt_new,dydt_new);
}
// Line ==============================================================

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
