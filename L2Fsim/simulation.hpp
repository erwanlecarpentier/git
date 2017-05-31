#ifndef L2FSIM_SIMULATION_HPP_
#define L2FSIM_SIMULATION_HPP_

#include <L2Fsim/flight_zone/flight_zone.hpp>
#include <L2Fsim/aircraft/aircraft.hpp>
#include <L2Fsim/stepper/stepper.hpp>
#include <L2Fsim/pilot/pilot.hpp>
#include <L2Fsim/utils/utils.hpp>

/**
 * @file simulation.hpp
 * @brief Simulation environment - Every choices concerning the simulation environment e.g. aircraft, pilot, environment etc. are made in the initialization of a 'simulation' object
 * @version 1.1
 * @since 1.0
 */

namespace L2Fsim {

class simulation {
public:
	/**
	 * @brief Attributes
	 * @param {flight_zone *} fz; pointer to a flight zone
	 * @param {aircraft *} ac; pointer to an aircraft
	 * @param {stepper *} st; pointer to a stepper
	 * @param {pilot *} pl; pointer to a pilot
	 */
	flight_zone *fz;
	aircraft *ac;
	stepper *st;
	pilot *pl;
	std::string ac_save_path;
	std::string fz_save_path;

	/**
	 * @brief Stepping function
	 * @param {double &} current_time; time at which the step is performed
	 * @param {const double &} time_step_width; width of the time step
     * @param {bool &} eos; end of simulation, the simulation reached the bounds of its model and must be stopped (e.g. limit of aircraft model validity)
	 */
	void step(double &current_time, const double &time_step_width, bool &eos) {
		(*st)(*fz, *ac, *pl, current_time, time_step_width, eos);
	}

    /** @brief Saving function, called at each time step */
	void save() {
        save_vector(ac->get_save(),ac_save_path,std::ofstream::app);

        std::vector<double> w;
        fz->wind(
            ac->get_state().getx(),
            ac->get_state().gety(),
            ac->get_state().getz(),
            ac->get_state().gett(),w);
        save_vector(w,fz_save_path,std::ofstream::app);
	}

    /** @brief Clear saving files (called before the simulation) */
	void clear_saves() {
        std::ofstream of;
        of.open(ac_save_path,std::ofstream::trunc);
        of.close();
        of.open(fz_save_path,std::ofstream::trunc);
        of.close();
	}
};

}

#endif
