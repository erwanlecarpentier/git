#ifndef L2FSIM_AIRCRAFT_HPP_
#define L2FSIM_AIRCRAFT_HPP_

#include <L2Fsim/flight_zone/flight_zone.hpp>
#include <L2Fsim/aircraft/state.hpp>
#include <L2Fsim/aircraft/command.hpp>
#include <vector>
#include <iostream>

/**
 * The abstract class Aircraft implementing the aircraft models
 * @version 1.1
 * @since 1.0
  *
 * An aircraft holds three important concepts:
 * - it has an internal state s which characterizes uniquely the aircraft's configuration at a given time;
 * - it has a dynamics function f, such that, given a command u, x'=f(x,u);
 * - @deprecated it has an observation function on the current state.
 */

namespace L2Fsim {

class aircraft {
public:
    virtual ~aircraft() = default;

	/**
     * @deprecated
	 * Updates the aircraft's state derivative vector
     * @param st the aircraft's state vector
     * @param cd the aircraft's command vector: alpha, beta, sigma
     * @param fz the flight zone
     * @param t the time
     * @param statedot the derivatives vector
	 */
    /*
	virtual const aircraft& state_dynamics(
        const std::vector<double> &st,
        const std::vector<double> &cd,
        flight_zone& fz,
        double t,
        std::vector<double> &statedot) const = 0;
    */

	/**
     * Compute the time derivative of the input state
     * @param {const flight_zone &} fz; flight zone
     * @param {const double} t; time
     * @param {state &} _s; updated state
     * @warning may implement a dynamic cast from state to a derived class
     */
	virtual aircraft & update_state_dynamic(flight_zone &fz, const double t, state &_s) = 0;

	/** Apply the command i.e. modify the state attribute of the aircraft accordingly to the command */
	virtual aircraft & apply_command() = 0;

	/** Get a reference on the actual state */
	virtual state & get_state() = 0;

	/** Get a reference on the actual command */
	virtual command & get_command() = 0;

	/**
     * @deprecated
     * Set the state
     * @param observation; the observation
     */
	//virtual aircraft& set_state(const std::vector<double> &state) = 0;

	/** Get the distance to the ceter of the map */
	virtual double get_distance_to_center() = 0;

    /** Check if the state vector contains values that are out of the model's range of validity */
    virtual aircraft& is_in_model() = 0;
};

}

#endif
