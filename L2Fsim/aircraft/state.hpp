#ifndef L2FSIM_STATE_HPP_
#define L2FSIM_STATE_HPP_

#include <fstream>

/**
 * @file state.hpp
 * @version 1.0
 * @since 1.0
 * @brief The abstract class state providing general access to the state variables of an aircraft
 */

namespace L2Fsim {

class state {
public:
    /** @brief Destructor */
    virtual ~state() = default;

    /** @brief Set time variable */
    virtual void set_time(double t) = 0;

    /**
     * @fn virtual double getx() = 0; @brief Get x coordinate in earth frame
     * @fn virtual double gety() = 0; @brief Get y coordinate in earth frame
     * @fn virtual double getz() = 0; @brief Get z coordinate in earth frame
     * @fn virtual double gett() = 0; @brief Get time coordinate
    */
    virtual double getx() = 0;
    virtual double gety() = 0;
    virtual double getz() = 0;
    virtual double gett() = 0;

    /**
     * @brief Dynamically creates a copy of the state
     * @warning dynamic allocation: delete the duplicated object
     * @return a pointer to the copy
     */
    virtual state * duplicate() const = 0;

    /** @brief Set every dynamic variables to 0 */
    virtual void clear_dynamic() = 0;

    /**
     * @brief Set the dynamic components i.e. the time derivatives interacting with the simulation integrator
     * @param {state &} s; state from which the dynamic components are copied
     * @warning may implement a dynamic cast from state to a derived class
     */
    virtual void set_dynamic(state &_s) = 0;

    /**
     * @brief Add the dynamic of a state to the current state
     * @param {state &} s; state from which the dynamic components are added
     * @param {double} coef; a multiplicative coefficient
     * @warning may implement a dynamic cast from state to a derived class
     */
    virtual void add_to_dynamic(state &_s, double coef) = 0;

    /**
     * @brief Apply a first order dynamic transition based on the values of the dynamic attributes (time derivatives)
     * @param {double} dt; time step
     */
    virtual void apply_dynamic(double dt) = 0;

    /**
     * @brief Get a vector containing the saved variables
     * @return {std::vector<double>}
     */
    virtual std::vector<double> get_save() = 0;

    /** @brief Return true if the state exceeds the admissible boundaries */
    virtual bool is_out_of_bounds() = 0;

    virtual void print() = 0;
};

}

#endif
