#ifndef L2FSIM_GAUSSIAN_PROCESS_HPP_
#define L2FSIM_GAUSSIAN_PROCESS_HPP_

#include <cmath>
#include <cassert>
#include <Eigen/Dense>

/**
 * @file gaussian_process.hpp
 * @brief A Gaussian Processes class implementing regression between inputs of
 * type 'std::vector<double>' and outputs of type 'double'.
 * @version 1.0
 */

namespace L2Fsim {

class gaussian_process {
	/**
	 * @brief Attributes
	 * @param {std::vector<std::vector<double>>} xdat; input data vector
	 * @param {std::vector<double>} ydat; output data vector
	 * @param {Eigen::MatrixXd} cov_matrix; covariance matrix
	 * @param {double} noise_var; noise variance
	 * @param {double (*) (std::vector<double>,std::vector<double>)} pointer to kernel
	 * function
	 */
	double (*kernel_function) (std::vector<double>,std::vector<double>);
	double noise_var;
	std::vector<std::vector<double>> xdat;
	std::vector<double> ydat;
	Eigen::MatrixXd cov_matrix;

	/**
	 * @brief Increase the size of the covariance matrix of 1
	 * @warning Called after any data point adding
	 */
	void update_cov_matrix() {
		unsigned int r = cov_matrix.rows();
		unsigned int c = cov_matrix.cols();
		unsigned int N = xdat.size();
		assert(r == c);
		assert(r == N - 1);
		cov_matrix.conservativeResize(r+1,c+1);
		for (unsigned int i=0; i<r; ++i) {
			cov_matrix(i,c) = kernel_function(xdat.at(i),xdat.at(r));
			cov_matrix(r,i) = cov_matrix(i,c);
		}
		cov_matrix(r,r) = kernel_function(xdat.at(r),xdat.at(r)) + noise_var;
	}

public:

	/** @brief Public constructor */
	gaussian_process(
		double (*_kernel_function) (std::vector<double>,std::vector<double>),
		double _noise_var = 1.) :
		kernel_function(_kernel_function),
		noise_var(_noise_var)
	{}

	/**
	 * @brief Append a data set to the data set and update the covariance matrix
	 * @param {const std::vector<std::vector<double>> &} x; vector of inputs
	 * @param {const std::vector<double> &} y; vector of outputs
	 */
	void add_data_set(
		const std::vector<std::vector<double>> &x,
		const std::vector<double> &y)
	{
		assert(x.size() == y.size());
		for (unsigned int i=0; i<x.size(); ++i) {
			add_point(x[i],y[i]);
		}
	}

	/**
	 * @brief Append a single point to the data set and update the covariance matrix
	 * @param {const std::vector<double> &} x; input
	 * @param {const double} y; output
	 */
	void add_point(
		const std::vector<double> &x,
		const double y)
	{
		xdat.push_back(x);
		ydat.push_back(y);
		update_cov_matrix();
	}

	/**
	 * @brief Predict the mean at a certain input
	 * @param {const std::vector<double> &} x; input
	 */
	double predict_mean(const std::vector<double> &x) {
		unsigned int sz = xdat.size();
		double* ptr = &ydat[0];
		Eigen::Map<Eigen::VectorXd> ydat2(ptr, sz);
		Eigen::VectorXd k(sz);
		for(unsigned int i=0; i<sz; ++i) {
			k(i) = kernel_function(x,xdat[i]);
		}
		return k.adjoint()*cov_matrix.inverse()*ydat2;
	}

	/**
	 * @brief Predict the variance at a certain input
	 * @param {const std::vector<double> &} x; input
	 */
	double predict_variance(const std::vector<double> &x) {
		unsigned int sz = xdat.size();
		Eigen::VectorXd k(sz);
		for(unsigned int i=0; i<sz; ++i) {
			k(i) = kernel_function(x,xdat[i]);
		}
		return kernel_function(x,x) - k.adjoint()*cov_matrix.inverse()*k;
	}

	std::vector<std::vector<double>> get_xdat() {return xdat;}
	std::vector<double> get_ydat() {return ydat;}

	void print_cov_mat() {
		std::cout << cov_matrix << std::endl;
	}
};

/** @brief Gaussian Kernel */
double gaussian_kernel(
	std::vector<double> a,
	std::vector<double> b)
{
	const unsigned int sz = a.size();
	assert(sz == b.size());
	Eigen::VectorXd a_b(sz);
	for(unsigned int i=0; i<sz; ++i) {
		a_b(i) = a[i] - b[i];
	}
	Eigen::MatrixXd M = 1e-3 * Eigen::MatrixXd::Identity(sz,sz); // precision matrix - hyperparameter
	return exp(-.5*a_b.adjoint()*M*a_b);
}

/** @brief Polynomial Kernel */
double polynomial_kernel(
	std::vector<double> a,
	std::vector<double> b)
{
	assert(a.size() == b.size());
	double delta = 0., deg = 2.; // hyperparameters
	double dot_product = 0.;
	for(unsigned int i=0; i<a.size(); ++i) {
		dot_product += a[i] * b[i];
	}
	return pow(dot_product + delta, deg);
}

}

#endif
