#ifndef L2FSIM_SIMULATION_HPP_
#define L2FSIM_SIMULATION_HPP_

#include <flight_zone.hpp>
#include <aircraft.hpp>
#include <stepper.hpp>
#include <pilot.hpp>
#include <utils.hpp>
#include <save.hpp>
#include <memory>

namespace L2Fsim {

/**
 * @brief Simulation environment
 *
 * @file simulation.hpp
 * @version 1.1
 * @since 1.0
 * This is the simulation environment. Every choices concerning the simulation environment
 * e.g. aircraft, pilot, environment etc. are made in the initialization of a 'simulation'
 * object.
 */
class simulation {
public:
	std::unique_ptr<flight_zone> fz; ///< Unique pointer to a flight zone.
	std::unique_ptr<aircraft> ac; ///< Unique pointer to an aircraft.
	std::unique_ptr<stepper> st; ///< Unique pointer to a stepper.
	std::unique_ptr<pilot> pl; ///< Unique pointer to a pilot.
	std::string st_log_path; ///< Log file path.
	std::string fz_log_path; ///< Log file path.

	/**
	 * @brief Stepping function
	 *
	 * @param {double &} current_time; time at which the step is performed
	 * @param {const double} time_step_width; width of the time step
     * @param {bool &} eos; end of simulation, the simulation reached the bounds of its model
     * and must be stopped (e.g. limit of aircraft model validity)
	 */
	void step(double &current_time, const double time_step_width, bool &eos) {
		(*st)(*fz, *ac, *pl, current_time, time_step_width, eos);
	}

    /**
     * @brief Saving method
     *
     * Saving method, called at each time step.
     */
	void save() {
        save_vector(ac->get_save(),st_log_path," ",std::ofstream::app);

        std::vector<double> w;
        fz->wind(
            ac->get_state().getx(),
            ac->get_state().gety(),
            ac->get_state().getz(),
            ac->get_state().gett(),w);
        save_vector(w,fz_log_path," ",std::ofstream::app);
	}

    /**
     * @brief Clear backup files
     *
     * Clear backup files (called at the beginning of the simulation).
     */
	void clear_saves() {
        std::ofstream of;
        of.open(st_log_path,std::ofstream::trunc);
        of.close();
        of.open(fz_log_path,std::ofstream::trunc);
        of.close();
	}
};

}

#endif // L2FSIM_SIMULATION_HPP_
