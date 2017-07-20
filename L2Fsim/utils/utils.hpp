#ifndef L2FSIM_UTILS_HPP_
#define L2FSIM_UTILS_HPP_

#include <random>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cassert>

/**
 * @file utils.hpp
 * @brief Helpful methods for 'l2f' project
 * @version 1.1
 * @since 1.0
 */

namespace L2Fsim {

constexpr double TO_RAD = 0.01745329251;
constexpr double COMPARISON_THRESHOLD = 1e-6;

inline void pop(){std::cout<<"pop"<<std::endl;}

/** @brief Return true if a == b up to a certain precision */
template <class T1, class T2>
constexpr bool is_equal_to(T1 a, T2 b) {
    return std::fabs(a-b)<COMPARISON_THRESHOLD;
}

/** @brief Return true if a < b  up to a certain precision */
template <class T1, class T2>
constexpr bool is_less_than(T1 a, T2 b) {
    return a<(b-COMPARISON_THRESHOLD);
}

/** @brief Return true if a > b  up to a certain precision */
template <class T1, class T2>
constexpr bool is_greater_than(T1 a, T2 b) {
    return a>(b+COMPARISON_THRESHOLD);
}

/**
 * @brief Sort the indices of the input vector, not depending on its content type
 * @param {const std::vector<double> &} v; input vector
 * @param {std::vector<unsigned int> &} up_ind; indices of the elements of v reaching the maximum values
 * @param {std::vector<unsigned int> &} dw_ind; indices of the other elements of v
 * @note template method
 */
template <class T>
inline void sort_indices(
    const std::vector<T> &v,
    std::vector<unsigned int> &up_ind,
    std::vector<unsigned int> &dw_ind)
{
    double maxval = *std::max_element(v.begin(),v.end());
    for (unsigned int j=0; j<v.size(); ++j) {
        //if(v[j] < maxval) {dw_ind.push_back(j);} // outdated: incorrect comparison between float, double, etc.
        if(is_less_than(v[j],maxval)) {dw_ind.push_back(j);}
        else {up_ind.push_back(j);}
    }
}

/**
 * @brief Pick a random indice of the input vector, not depending on its content type
 * @param {const std::vector<T> &} v; input vector
 * @return {unsigned int} a random indice
 * @note Initialize a random seed when executing your program
 * @note Template method
 */
template <class T>
inline unsigned int rand_indice(const std::vector<T> &v) {
    assert(v.size() != 0);
    return rand() % v.size();
}

/**
 * @brief Pick a random element of the input vector, not depending on its content type
 * @param {const std::vector<T> &} v; input vector
 * @return {int} a random indice
 * @note initialize a random seed when executing your program
 * @note template method
 */
template <class T>
inline T rand_element(const std::vector<T> &v) {
    return v.at(rand_indice(v));
}

template <class T>
inline void shuffle(std::vector<T> &v) {
    std::random_shuffle(v.begin(), v.end());
}

/** @brief Return the sign of the input double */
inline double sgn(double x) {
    return is_less_than(x,0.) ? -1. : 1.;
}

/** @return a uniformly picked double in range [fmin,fmax] */
inline double rand_double(double fmin, double fmax) {
    return (rand() / (double)RAND_MAX) * (fmax-fmin) + fmin;
}

/** @return a uniformly picked int in range [fmin,fmax] */
inline int rand_int(int fmin, int fmax) {
    return (rand() % (fmax-fmin) + fmin);
}

/**
 * @brief Get the indice of the maximum element in the input vector, ties are broken randomly
 * @param {const std::vector<T> &} v; input vector
 * @return Indice of the maximum element in the input vector
 * @note template method
 */
template <class T>
inline unsigned int argmax(const std::vector<T> &v) {
    double maxval = *std::max_element(v.begin(),v.end());
    std::vector<unsigned int> up_ind;
    for (unsigned int j=0; j<v.size(); ++j) {
        if(!is_less_than(v[j],maxval)) {up_ind.push_back(j);}
    }
    return rand_element(up_ind);
}

/** @brief See 'argmax' method */
template <class T>
inline unsigned int argmin(const std::vector<T> &v) {
    double minval = *std::min_element(v.begin(),v.end());
    std::vector<unsigned int> lo_ind;
    for (unsigned int j=0; j<v.size(); ++j) {
        if(!is_greater_than(v[j],minval)) {lo_ind.push_back(j);}
    }
    return rand_element(lo_ind);
}

/**
 * @brief Save a vector into a file
 * @note template method
 * @param {const std::vector<content_type> &} v; input vector, can contain any implementation-defined type
 * @param {const std::string &} path; path to output file
 * @param {std::ofstream::openmode} mode; writing mode, default overrides
 */
template <class content_type>
inline void save_vector(
    const std::vector<content_type> &v,
    const std::string &path,
    std::ofstream::openmode mode = std::ofstream::out)
{
    std::ofstream outfile;
    outfile.open(path,mode);
    for (unsigned int i=0; i<v.size(); ++i) {
        outfile << v[i] << " ";
    }
    outfile << std::endl;
    outfile.close();
}

/**
 * @brief Sigmoid function
 * @param {const double &} x; point to evaluate the sigmoid
 * @param {const double &} x_max; point the sigmoid is 0.99
 * @param {const double &} x_middle; point the sigmoid is 0.5
 */
inline double sigmoid(const double &x, const double &x_max, const double &x_middle) {
    return 1./(1. + exp(-(x-x_middle)/(x_max*.217622180186)));
}

}

#endif
