#ifndef L2FSIM_AIRCRAFT_HPP_
#define L2FSIM_AIRCRAFT_HPP_

#include <flight_zone.hpp>
#include <state.hpp>
#include <command.hpp>
#include <vector>
#include <iostream>

/**
 * @brief The abstract class Aircraft implementing the aircraft models
 * @version 1.1
 * @since 1.0
 *
 * An aircraft holds three important concepts:
 * - it has an internal state s which characterizes uniquely the aircraft's configuration at a given time;
 * - it has a dynamics function f, such that, given a command u, x'=f(x,u).
 */

namespace L2Fsim {

class aircraft {
public:
    virtual ~aircraft() = default;

	/**
     * @brief Compute the time derivative of the input state
     * @param {const flight_zone &} fz; flight zone
     * @param {const double} t; time
     * @param {state &} _s; updated state
     * @warning may implement a dynamic cast from state to a derived class
     */
	virtual aircraft & update_state_dynamic(flight_zone &fz, const double t, state &_s) = 0;

	/** @brief Apply the command i.e. modify the state attribute of the aircraft accordingly to the command */
	virtual aircraft & apply_command() = 0;

	/** @brief Get a reference on the actual state */
	virtual state & get_state() = 0;

	/** @brief Get a reference on the actual command */
	virtual command & get_command() = 0;

	/** @brief Get the distance to the ceter of the map */
	virtual double get_distance_to_center() = 0;

    /**
     * @brief Check if the state vector contains values that are out of the model's range of validity
     * @return true if the aircraft still is in its validity model
     */
    virtual bool is_in_model() = 0;

    /** @brief Return the saved data at each time step */
    virtual std::vector<double> get_save() = 0;
};

}

#endif
