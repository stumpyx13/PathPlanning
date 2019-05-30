#ifndef ENVIRONMENT_CLASSES_H
#define ENVIRONMENT_CLASSES_H
#include "std_lib_facilities.h"

class Obstacle;
class Point;
class Environment;
class Line;

class Environment{
	double deltaX = 0; //size in the x direction
	double deltaY = 0; //size in the y direction
	std::vector<std::shared_ptr<Obstacle>> obstacleList; // list of obstacles in the environment
	public:
		// constructor
		Environment(double setdeltaX, double setdeltaY);
		// member functions
		void addObstacle(std::shared_ptr<Obstacle> obs); // Add a single obstacle to the environment
		std::vector<std::shared_ptr<Obstacle>> getObstacleList() const; // return vector with obstacles
		bool obstacleFree(std::shared_ptr<Point> p) const; // check if a point is in an obstacle free region
		double getDeltaX() const;
		double getDeltaY() const;

		void printItem(std::ofstream& os) const;
		double getMaxDistance() const;

		void generateRandomObstacles(const int N_obstacles,
				const double sizeBound);

		bool inEnvironment(std::shared_ptr<Point> p) const;
};
class Obstacle{
	protected:
		double x = 0; //x-position of the top left corner
		double y = 0; //y-position of the top left corner
		double dx = 0; // length of obstacle in the x direction to the right
		double dy = 0; // length of obstacle in the y direction to the bottom
		std::vector<std::shared_ptr<Point>> pointList =
			std::vector<std::shared_ptr<Point>>(4);
		std::vector<std::shared_ptr<Line>> lineList =
			std::vector<std::shared_ptr<Line>>(4);
	public:
		//constructors
		Obstacle(){}
		Obstacle(const double xMax, const double yMax, const double sizeBound);
		Obstacle(double xSet, double ySet, double dxSet, double dySet);
		// member functions
		bool inObstacle(std::shared_ptr<Point> p) const; // Check if point is in obstacle
		bool lineIntersects(const std::shared_ptr<Line> p_line) const;
		double getdx() const;
		double getdy() const;
		double getX() const;
		double getY() const;

		void printItem(std::ofstream& os) const;
		void genRandom(const double xMax, const double yMax,
				const double sizeBound);
};
class Point{ // just position
	protected:
		double x = 0;
		double y = 0;
	public:
		// constructor
		Point();
		Point(double xSet, double ySet);

		// member functions
		void genRandom(const Environment& env);
		double getX() const; // get x point
		double getY() const; // get y point
		void setX(double xSet); // set x coordinate
		void setY(double ySet);// set y coordinate
		void printItem(std::ofstream& os) const; // print point coordinates on single line
		double calculateDistance(const std::shared_ptr<Point> p) const;
		double calculateCost(const std::shared_ptr<Point> p) const;
		std::shared_ptr<Point> moveTowards(const std::shared_ptr<Point> p_goal,const double dist) const;
};

class DynamicPoint : public Point { // includes Point "dynamics"
	double dxdt = 0;
	double dydt = 0;
	double dxdtMax = 5;
	double dydtMax = 5;
	double d2xdt2 = 0;
	double d2ydt2 = 0;

	double accel_max = 1; // hard_coded for now, need to include in member functions

	public:
		DynamicPoint();
		DynamicPoint(double xSet, double ySet, double dxdtSet, double dydtSet,
			double accel_max_in);

		// override Point functions
		void genRandom(const Environment& env);
		double calculateDistance(const std::shared_ptr<DynamicPoint> p) const;
		double calculateCost(const std::shared_ptr<DynamicPoint> p) const;
		std::shared_ptr<DynamicPoint> moveTowards(const std::shared_ptr<DynamicPoint> p_goal,
			const double dt) const;

		// new member functions
		double get_dxdtMax() const;
		double get_dydtMax() const;
		void set_dxdtMax(double dxdtMax_in);
		void set_dydtMax(double dydtMax_in);

		double get_dxdt() const;
		double get_dydt() const;
		void set_dxdt(double dxdt_in);
		void set_dydt(double dydt_in);
		void printItem(std::ofstream& os) const;
};

class Line{
	protected:
		std::shared_ptr<Point> p1;
		std::shared_ptr<Point> p2;
	public:
		Line();
		Line(std::shared_ptr<Point> p1_in,
				std::shared_ptr<Point> p2_in);
		std::shared_ptr<Point> getPoint1() const;
		std::shared_ptr<Point> getPoint2() const;
		void setPoint1(std::shared_ptr<Point> p1_in);
		void setPoint2(std::shared_ptr<Point> p2_in);
		void setPoints(std::shared_ptr<Point> p1_in,
				std::shared_ptr<Point> p2_in);
		bool intersectCheck(std::shared_ptr<Line> p_check) const;
};

#endif
