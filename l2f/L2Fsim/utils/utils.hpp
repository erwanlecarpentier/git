#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifndef L2FSIM_UTILS_HPP_
#define L2FSIM_UTILS_HPP_

#include <random>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace L2Fsim {

/**
 * @brief Sort the indices of the input vector, not depending on its content type
 * @param {const std::vector<double> &} v; input vector
 * @param {std::vector<unsigned int> &} up_ind; indices of the elements of v reaching the maximum values
 * @param {std::vector<unsigned int> &} dw_ind; indices of the other elements of v
 */
template <class content_type>
inline void sort_indices(const std::vector<content_type> &v,
    std::vector<unsigned int> &up_ind,
    std::vector<unsigned int> &dw_ind)
{
    double maxval = *std::max_element(v.begin(),v.end());
    for (unsigned int j=0; j<v.size(); ++j) {
        if(v[j]<maxval) {dw_ind.push_back(j);}
        else {up_ind.push_back(j);}
    }
}

/**
 * @brief Overload of 'sort_indices' function only retrieving the indices of the elements of v reaching the maximum values
 * @param {const std::vector<double> &} v; input vector
 * @param {std::vector<unsigned int> &} up_ind; indices of the elements of v reaching the maximum values
 */
template <class content_type>
inline void sort_indices(const std::vector<content_type> &v,
    std::vector<unsigned int> &up_ind)
{
    double maxval = *std::max_element(v.begin(),v.end());
    for (unsigned int j=0; j<v.size(); ++j) {
        if(v[j]>=maxval) {up_ind.push_back(j);}
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

inline double sgn(double T) {
    if (T > 0){ return 1.0;}
    if (T < 0){ return -1.0;}
    if (T == 0){ return 1.0;}
    return 0;
}

/** @return a uniformly picked double in range [fmin,fmax] */
inline double rand_double(double fmin, double fmax) {
    return ( rand()/(double)RAND_MAX ) * (fmax-fmin) + fmin;
}

/**
 * return an int between int "a" and int "b"
 * @note implicitly a<b
 */
/** @return a uniformly picked int in range [fmin,fmax] */
inline int rand_int(int fmin, int fmax) {
    return (rand()%(fmax-fmin) +fmin);
}

/** produce a number following a normal distribution */
inline double normalLaw() {
    double W,V1,V2;
    do
    {
        double U1=rand_double(0., 1.);
        double U2=rand_double(0., 1.);
        V1=2*U1-1;
        V2=2*U2-1;
        W=V1*V1+V2*V2;
    }while(W>1);
    double W_function=sqrt(-2*log(W)/W);
    return V1*W_function;
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

inline double sigmoid(double x,double a,double c) {
    double exp_value;
    float return_value;
    exp_value = exp( -1/a*(x-c));
    return_value = 1 / (1 + exp_value);
    return return_value;
}

}

#endif
