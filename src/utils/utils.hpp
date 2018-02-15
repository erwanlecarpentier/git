#ifndef L2FSIM_UTILS_HPP_
#define L2FSIM_UTILS_HPP_

#include <random>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cassert>

/**
 * @brief Helpful functions
 */

namespace L2Fsim {

constexpr double TO_RAD = 0.01745329251;
constexpr double COMPARISON_THRESHOLD = 1e-6;

/**
 * @brief Print
 *
 * Generic printing method. Template method.
 */
template <class T>
void print(T t) {
	std::cout << t << std::endl;
}

/**
 * @brief Print
 *
 * Generic printing method for containers. Template method.
 */
template <class T>
void printv(std::vector<T> v) {
	for (auto &elt : v) {
		std::cout << elt << " ";
	}
	std::cout << std::endl;
}

inline void pop(){std::cout<<"pop"<<std::endl;}

/**
 * @brief Equality comparison
 *
 * Test if the input arguments are equal.
 * Template method.
 * @param {T1} a; 1st argument
 * @param {T2} b; 2nd argument
 * @param {double} precision; precision of the test
 * @return Return true if the input arguments are equal.
 */
template <class T1, class T2>
constexpr bool are_equal(const T1 a, const T2 b, double precision = COMPARISON_THRESHOLD) {
    return std::fabs(a-b)<precision;
}

/**
 * @brief Strict inferiority comparison
 *
 * Test if the 1st input arguments is strictly inferior to the 2nd one.
 * Template method.
 * @param {T1} a; 1st argument
 * @param {T2} b; 2nd argument
 * @param {double} precision; precision of the test
 * @return Return true if the 1st input arguments is strictly inferior to the 2nd one.
 */
template <class T1, class T2>
constexpr bool is_less_than(const T1 a, const T2 b, double precision = COMPARISON_THRESHOLD) {
    return a<(b-precision);
}

/**
 * @brief Strict superiority comparison
 *
 * Test if the 1st input arguments is strictly superior to the 2nd one.
 * Template method.
 * @param {T1} a; 1st argument
 * @param {T2} b; 2nd argument
 * @param {double} precision; precision of the test
 * @return Return true if the 1st input arguments is strictly superior to the 2nd one.
 */
template <class T1, class T2>
constexpr bool is_greater_than(const T1 a, const T2 b, double precision = COMPARISON_THRESHOLD) {
    return a>(b+precision);
}

/**
 * @brief Sign function
 *
 * Template method.
 * @return Return -1. if x < 0., +1. else
 */
template <class T>
double sign(T x) {
    return (is_less_than(x,0.)) ? -1. : 1.;
}

/**
 * @brief Shuffle
 *
 * Shuffle the content of the vector randomly. Template method.
 */
template <class T>
inline void shuffle(std::vector<T> &v) {
    std::random_shuffle(v.begin(), v.end());
}

/**
 * @brief Sort indices
 *
 * Sort the indices of the input vector. Template method.
 * @param {const std::vector<double> &} v; input vector
 * @param {std::vector<unsigned int> &} up_ind; indices of the elements of v
 * reaching the maximum values
 * @param {std::vector<unsigned int> &} dw_ind; indices of the other elements of v
 */
template <class T>
inline void sort_indices(
    const std::vector<T> &v,
    std::vector<unsigned int> &up_ind,
    std::vector<unsigned int> &dw_ind)
{
    double maxval = *std::max_element(v.begin(),v.end());
    for (unsigned int j=0; j<v.size(); ++j) {
        if(is_less_than(v[j],maxval)) {dw_ind.push_back(j);}
        else {up_ind.push_back(j);}
    }
}

/**
 * @brief Random indice
 *
 * Pick a random indice of the input vector. You should initialize a random seed when
 * executing your program. Template method.
 * @param {const std::vector<T> &} v; input vector
 * @return Return a random indice.
 */
template <class T>
inline unsigned int rand_indice(const std::vector<T> &v) {
    assert(v.size() != 0);
    return rand() % v.size();
}

/**
 * @brief Random element
 *
 * Pick a random element of the input vector. You should initialize a random seed when
 * executing your program. Template method.
 * @param {const std::vector<T> &} v; input vector
 * @return Return a random element.
 */
template <class T>
inline T rand_element(const std::vector<T> &v) {
    return v.at(rand_indice(v));
}

/**
 * @brief Remove elements
 *
 * Remove the given elements of the input vector.
 * @param {std::vector<T> &} v; input vector
 * @param {std::vector<unsigned> &} indices; indices of the vectors to be removed
 */
template <class T>
void remove_elements(std::vector<T> &v, std::vector<unsigned> &indices) {
    for (unsigned i=0; i<indices.size(); ++i) {
        v.erase(v.begin() + indices[i]);
        for(unsigned j=i+1; j<indices.size(); ++j) {
            --indices[j];
        }
    }
}

/**
 * @brief Argmax
 *
 * Get the indice of the maximum element in the input vector, ties are broken arbitrarily.
 * Template method.
 * @param {const std::vector<T> &} v; input vector
 * @return Return the indice of the maximum element in the input vector.
 */
template <class T>
inline unsigned argmax(const std::vector<T> &v) {
    auto maxval = *std::max_element(v.begin(),v.end());
    std::vector<unsigned> up_ind;
    for (unsigned j=0; j<v.size(); ++j) {
        if(!is_less_than(v[j],maxval)) {up_ind.push_back(j);}
    }
    return rand_element(up_ind);
}

/**
 * @brief Argmin
 *
 * See 'argmax' method. Template method.
 */
template <class T>
inline unsigned argmin(const std::vector<T> &v) {
    auto minval = *std::min_element(v.begin(),v.end());
    std::vector<unsigned> lo_ind;
    for (unsigned j=0; j<v.size(); ++j) {
        if(!is_greater_than(v[j],minval)) {lo_ind.push_back(j);}
    }
    return rand_element(lo_ind);
}

/**
 * @brief Uniformly distributed integer
 *
 * Generate a uniformly distributed integer
 * @return Return the sample
 */
int uniform_integer(int int_min, int int_max) {
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_int_distribution<int> distribution(int_min,int_max);
    return distribution(generator);
}

/**
 * @brief Uniformly distributed double
 *
 * Generate a uniformly distributed double
 * @return Return the sample
 */
double uniform_double(double double_min, double double_max) {
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<double> distribution(double_min,double_max);
    return distribution(generator);
}

/**
 * @brief Normally distributed double
 *
 * Generate a normally distributed double
 * @return Return the sample
 */
double normal_double(double mean, double stddev) {
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution<double> distribution(mean,stddev);
    return distribution(generator);
}

/**
 * @brief Sigmoid function
 *
 * @param {const double} x; point to evaluate the sigmoid
 * @param {const double} x_max; point the sigmoid is 0.99
 * @param {const double} x_middle; point the sigmoid is 0.5
 */
inline double sigmoid(const double x, const double x_max, const double x_middle = 0.) {
    return 1./(1. + exp(-(x-x_middle)/(x_max*.217622180186)));
}

}

#endif // L2FSIM_UTILS_HPP_
