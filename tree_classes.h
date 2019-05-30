#ifndef TREE_CLASSES_H
#define TREE_CLASSES_H
#include "std_lib_facilities.h"

template <typename T>
class TreeNode;
template <typename T>
class TreeAncestorPath;
// TreeNode represents the nodes in a search tree. Each TreeNode has an item that could be a Point, int, etc and possibly a parent TreeNode, should be the same item type. If no parent, assumes that the TreeNode is at the top of the tree (progenitor node).
template <typename T>
class TreeNode{
	protected:
		std::shared_ptr<T> item_p = nullptr;
		std::shared_ptr<TreeNode<T>> parent_p = nullptr;
		double cost = 0;
	public:
		TreeNode();
		TreeNode(std::shared_ptr<T> item_in);
		TreeNode(std::shared_ptr<T> item_in,
				std::shared_ptr<TreeNode<T>> parent_in);
		void setItem(std::shared_ptr<T> item_in);
		std::shared_ptr<T> getItem() const;
		void setParent(std::shared_ptr<TreeNode<T>> parent_in);
		std::shared_ptr<TreeNode<T>> getParent() const;
		double getCost() const;
		void setCost(const double newCost);
};

// TreeAncestorPath contains a vector that is the path from one TreeNode to the top of the Tree that it contains. It calculates the ancestor path by getting the parents of each TreeNode until it finds the TreeNode without a parent.
template <typename T>
class TreeAncestorPath{
	protected:
		std::vector<std::shared_ptr<TreeNode<T>>> path;
	public:
		TreeAncestorPath(const TreeNode<T>& node);
		void createAncestorPath(const TreeNode<T>& youngestNode);
		std::vector<std::shared_ptr<TreeNode<T>>> getPath() const;
		void printPath(std::ofstream& os) const;
		double getPathCost() const{return path.back()->getCost();}
};

#endif
