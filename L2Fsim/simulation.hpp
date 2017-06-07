#ifndef L2FSIM_SIMULATION_HPP_
#define L2FSIM_SIMULATION_HPP_

#include <memory>
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
	 * @param {std::unique_ptr<flight_zone>} fz; unique pointer to a flight zone
	 * @param {std::unique_ptr<aircraft>} ac; unique pointer to an aircraft
	 * @param {std::unique_ptr<stepper>} st; unique pointer to a stepper
	 * @param {std::unique_ptr<pilot>} pl; unique pointer to a pilot
	 * @param {std::string} st_log_path, fz_log_path; log file paths
	 */
	std::unique_ptr<flight_zone> fz;
	std::unique_ptr<aircraft> ac;
	std::unique_ptr<stepper> st;
	std::unique_ptr<pilot> pl;
	std::string st_log_path;
	std::string fz_log_path;

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
        save_vector(ac->get_save(),st_log_path,std::ofstream::app);

        std::vector<double> w;
        fz->wind(
            ac->get_state().getx(),
            ac->get_state().gety(),
            ac->get_state().getz(),
            ac->get_state().gett(),w);
        save_vector(w,fz_log_path,std::ofstream::app);
	}

    /** @brief Clear log files (called before the simulation) */
	void clear_saves() {
        std::ofstream of;
        of.open(st_log_path,std::ofstream::trunc);
        of.close();
        of.open(fz_log_path,std::ofstream::trunc);
        of.close();
	}
};

}

#endif
