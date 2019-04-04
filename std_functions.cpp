#include "std_lib_facilities.h"

template <typename T>
void check_lengths(std::vector<T> const &a, std::vector<T> const &b){
	if(a.size() != b.size()){
		throw std::domain_error("Vector lenghts not equal!");
	}
}


template <typename T>
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b){
	// Overloaded addition operator to support vector additon
	// Check vector lengths:
	check_lengths(a,b);
	
	// Initialize result vector
	std::vector<T> result;
	result.reserve(a.size()); // reserve result memory on the heap
	
	std::transform(a.begin(), a.end(), b.begin(),
			std::back_inserter(result),std::plus<T>());

	return result;
}
