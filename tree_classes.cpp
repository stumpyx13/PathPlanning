#ifndef TREE_CLASSES_CPP
#define TREE_CLASSES_CPP
#include "tree_classes.h"

// TreeNode constructors
template <typename T>
TreeNode<T>::TreeNode(){}

template <typename T>
TreeNode<T>::TreeNode(std::shared_ptr<T> item_in){ //intended for top of the tree node with no parent
	item_p = item_in;
}

template <typename T>
TreeNode<T>::TreeNode(std::shared_ptr<T> item_in, std::shared_ptr<TreeNode<T>> parent_in){ //intended for nodes with parents
	item_p = item_in;
	parent_p = parent_in;
}

template <typename T>
// TreeNode member functions
void TreeNode<T>::setItem(std::shared_ptr<T> item_in){ // set the item that the TreeNode contains
	item_p = item_in;
}

template <typename T>
std::shared_ptr<T>  TreeNode<T>::getItem() const{ // get the item stored in the TreeNode
	return item_p;
}

template <typename T>
void TreeNode<T>::setParent(std::shared_ptr<TreeNode<T>> parent_in){ // set the TreeNode parent
	parent_p = parent_in;
}

template <typename T>
std::shared_ptr<TreeNode<T>> TreeNode<T>::getParent() const{
	// get the TreeNode parent
	return parent_p;
}

template<typename T>
double TreeNode<T>::getCost() const{
	return cost;
}

template<typename T>
void TreeNode<T>::setCost(const double newCost){
	cost = newCost;
}

template<typename T>
// TreeAncestorPath member functions
TreeAncestorPath<T>::TreeAncestorPath(const TreeNode<T>& node){
	createAncestorPath(node);
}

template <typename T>
void TreeAncestorPath<T>::createAncestorPath(const TreeNode<T>& youngestNode){ //create the path from input TreeNode to top of the Tree
	path.insert(path.begin(),
			std::make_shared<TreeNode<T>>(youngestNode));
	std::shared_ptr<TreeNode<T>> parent = youngestNode.getParent();
	while(parent != nullptr) {
		path.insert(path.begin(),parent);
		std::shared_ptr<TreeNode<T>> currentNode = parent;
		parent = currentNode->getParent();
	}
}

template <typename T>
std::vector<std::shared_ptr<TreeNode<T>>> TreeAncestorPath<T>::getPath() const{ // get the stored ancestor path
	return path;
}

template<typename T>
void TreeAncestorPath<T>::printPath(std::ofstream& os) const{
	for(auto p_node : path){
		auto nodeItem = p_node -> getItem();
		nodeItem->printItem(os);
		os << "\n";
	}
}
#endif
