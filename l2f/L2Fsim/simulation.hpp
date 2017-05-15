#ifndef L2FSIM_SIMULATION_HPP_
#define L2FSIM_SIMULATION_HPP_

#include <L2Fsim/flight_zone/flight_zone.hpp>
#include <L2Fsim/aircraft/aircraft.hpp>
#include <L2Fsim/stepper/stepper.hpp>
#include <L2Fsim/pilot/pilot.hpp>

/**
 * @file simulation.hpp
 * @brief Simulation environment
 * @version 1.1
 * @since 1.0
 *
 * Every choices concerning the simulation environment e.g. aircraft, pilot, environment etc.
 * are made in the initialization of a 'simulation' object
 */

namespace L2Fsim {

class simulation {
public:
	/**
	 * Attributes
	 * @param {flight_zone *} fz; pointer to a flight zone
	 * @param {aircraft *} ac; pointer to an aircraft
	 * @param {stepper *} st; pointer to a stepper
	 * @param {pilot *} pl; pointer to a pilot
	 */
	flight_zone *fz;
	aircraft *ac;
	stepper *st;
	pilot *pl;

	/**
	 * Stepping function
	 * @param {double &} current_time; time at which the step is performed
	 * @param {const double &} time_step_width; width of the time step
	 */
	void step(double &current_time, const double &time_step_width) {
		(*st)(*fz, *ac, *pl, current_time, time_step_width);
	}
};

}

#endif
