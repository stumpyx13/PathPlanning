#PathPlanning
Project used to test standard path planning algorithms (README current as of 4/3/19)
Currently contains 3 primary class files:
1) environment_classes: Contains class definitions for Environment, Obstacle, Point, and Line.
  a) Environment is a 2D representation of the environment, paramterized by the length along the x-direction (deltaX) and y-directon (deltaY). Bottom left-hand corner is assumed to lie at the origin (0,0). Has one vector containing shared_ptrs to the Obstacles in the Environment.
  b) Obstacle is a 2D, rectangular representation of an obstacle in the Environment that is considered solid and which cannot be passed through. It is parameterized by the position of the top left corner (x,y) and length in the x-direction (dx) and length in the y-direction (dy).
  c) Point is a 2D representation of an infinitesimal point in the Environment, paramterized by its position (x,y).
  d) Line is a 2D representation of a line in the Environment, paramterized by its two end Points.
  
2) tree_classes: Contains class definitions for TreeNode and TreeAncestorPath
  a) TreeNode: Represents a node in a tree, parameterized by a shared_ptr to the item that it contains, a shared_ptr to its TreeNode parent and cost to reach the node from the root. The root is assumed to have no parent (nullptr), while all other TreeNodes in a tree should have a parent.
  b) TreeAncestorPath: represents a path of TreeNodes, where the parent of each TreeNode is the TreeNode preceding it in the Path. It is parameterized by a vector of shared_ptrs pointing to TreeNodes.
  
3) RRT_classes: Contains a single class definition for RRT_star
  a) RRT_star: Represents the RRT* algorithm from a start to an end region in the Environment. It is paramterized by the number of desired sampled points (N_points), a vector of shared_ptrs to TreeNodes resulting from RRT*, an Environment (env), a goal region (represented by an Obstacle, goalRegion) and a start (T, where T is the item/point in the Environment).
  When the RRT_star constructor is invoked, the algorithm is run. Only one constructor is provided for this reason that requires all parameters as input.
