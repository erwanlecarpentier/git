#ifndef L2FSIM_STEPPER_HPP_
#define L2FSIM_STEPPER_HPP_

#include <pilot.hpp>

/**
 * @brief Stepper virtual class
 *
 * @file stepper.hpp
 * @version 1.1
 * @since 1.0
 */

namespace L2Fsim {

struct stepper {
public:
    virtual ~stepper() = default;

    /**
     * @brief Stepper method
     *
     * Temporal integrator of the model.
     * @param {flight_zone &} fz; flight zone
     * @param {aircraft &} ac; aircraft
     * @param {pilot &} pl; pilot
     * @param {double &} current_time; current time
     * @param {const double} time_step_width; period of time during which we perform integration
     * @param {bool &} eos; end of simulation, the simulation reached the bounds of its model and must be stopped (e.g. limit of aircraft model validity)
     */
	virtual void operator()(
        flight_zone &fz,
        aircraft &ac,
        pilot &pl,
        double &current_time,
        const double time_step_width,
        bool &eos) = 0;
};

}

#endif //L2FSIM_STEPPER_HPP_
