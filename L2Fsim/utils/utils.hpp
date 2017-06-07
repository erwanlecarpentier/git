#ifndef L2FSIM_UTILS_HPP_
#define L2FSIM_UTILS_HPP_

#include <random>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>

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
    return a<b-COMPARISON_THRESHOLD;
}

/** @brief Return true if a > b  up to a certain precision */
template <class T1, class T2>
constexpr bool is_greater_than(T1 a, T2 b) {
    return a>b+COMPARISON_THRESHOLD;
}

/**
 * @brief Sort the indices of the input vector, not depending on its content type
 * @param {const std::vector<double> &} v; input vector
 * @param {std::vector<unsigned int> &} up_ind; indices of the elements of v reaching the maximum values
 * @param {std::vector<unsigned int> &} dw_ind; indices of the other elements of v
 * @note template method
 */
template <class content_type>
inline void sort_indices(
    const std::vector<content_type> &v,
    std::vector<unsigned int> &up_ind,
    std::vector<unsigned int> &dw_ind)
{
    double maxval = *std::max_element(v.begin(),v.end());
    for (unsigned int j=0; j<v.size(); ++j) {
        if(v[j] < maxval) {dw_ind.push_back(j);}
        else {up_ind.push_back(j);}
    }
}

/**
 * @brief Get the vector of indices of 'v' for which 'v' content is maximum
 * @param {const std::vector<double> &} v; input vector
 * @param {std::vector<unsigned int> &} up_ind; indices of the elements of v reaching the maximum values
 * @note template method
 */
template <class content_type>
inline void argmax(
    const std::vector<content_type> &v,
    std::vector<unsigned int> &up_ind)
{
    double maxval = *std::max_element(v.begin(),v.end());
    for (unsigned int j=0; j<v.size(); ++j) {
        if(v[j] >= maxval) {up_ind.push_back(j);}
    }
}

/**
 * @brief Pick a random indice of the input vector, not depending on its content type
 * @param {const std::vector<content_type> &} v; input vector
 * @return {int} a random indice
 * @note initialize a random seed when executing your program
 * @note template method
 */
template <class content_type>
inline int rand_indice(const std::vector<content_type> &v) {
    if(v.size()==0)
    {std::cout<<"Error in function 'int L2Fsim::rand_indice': argument has size 0"<<std::endl;}
    return rand() % v.size();
}

/**
 * @brief Pick a random element of the input vector, not depending on its content type
 * @param {const std::vector<content_type> &} v; input vector
 * @return {int} a random indice
 * @note initialize a random seed when executing your program
 * @note template method
 */
template <class content_type>
inline content_type rand_element(const std::vector<content_type> &v) {
    return v.at(rand_indice(v));
}

/** @brief Return the sign of the input double */
inline double sgn(double T) {
    if (T >= 0) {return 1.;}
    else {return -1.;}
}

/** @return a uniformly picked double in range [fmin,fmax] */
inline double rand_double(double fmin, double fmax) {
    return (rand() / (double)RAND_MAX) * (fmax-fmin) + fmin;
}

/** @return a uniformly picked int in range [fmin,fmax] */
inline int rand_int(int fmin, int fmax) {
    return (rand()%(fmax-fmin) +fmin);
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

/** @brief Sigmoid function */
inline double sigmoid(double x, double a, double c) {
    return 1. / (1.+exp(-(x-c)/a));
}

}

#endif
