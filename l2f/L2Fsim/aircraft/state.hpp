#ifndef L2FSIM_STATE_HPP_
#define L2FSIM_STATE_HPP_

#include <fstream>

/**
 * The abstract class state providing general access to the state variables of an aircraft
 * @version 1.0
 * @since 1.0
 */

namespace L2Fsim {

class state {
public:
    virtual ~state() = default;

    /**
     * Dynamically creates a copy of the state
     * @warning dynamic allocation: delete the duplicated object
     * @return a pointer to the copy
     */
    virtual state * duplicate() const = 0;

    /** Set every dynamic variables to 0 */
    virtual void clear_dynamic() = 0;

    /**
     * Set the dynamic components i.e. the time derivatives interacting with the simulation integrator
     * @param {state &} s; state from which the dynamic components are copied
     * @warning may implement a dynamic cast from state to a derived class
     */
    virtual void set_dynamic(state &_s) = 0;

    /**
     * Add the dynamic of a state to the current state
     * @param {state &} s; state from which the dynamic components are added
     * @param {const double} coef; a multiplicative coefficient
     * @warning may implement a dynamic cast from state to a derived class
     */
    virtual void add_to_dynamic(state &_s, const double coef) = 0;

    /**
     * Apply a first order dynamic transition based on the values of the dynamic attributes (time derivatives)
     * @param {double} dt; time step
     */
    virtual void apply_dynamic(const double dt) = 0;

    /**
     * Update time variable
     * @param {const double &} t; current time
     */
    virtual void update_time(const double &t) = 0;

    /**
     * Save the state & the time step into a file
     * @param {std::string} filename; path of the log file
     */
    virtual void save(std::string filename) = 0;

    virtual void print() = 0;
};

}

#endif
