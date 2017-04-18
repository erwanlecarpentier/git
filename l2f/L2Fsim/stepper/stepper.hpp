#ifndef L2FSIM_STEPPER_HPP_
#define L2FSIM_STEPPER_HPP_

#include <L2Fsim/flight_zone/flight_zone.hpp>
#include <L2Fsim/aircraft/aircraft.hpp>
#include <L2Fsim/pilot/pilot.hpp>

namespace L2Fsim {

struct stepper {
public:
    virtual ~stepper() = default;

    /**
     * Temporal integrator of the model
     * @param {flight_zone &} fz; flight zone
     * @param {aircraft &} ac; aircraft
     * @param {pilot &} pl; pilot
     * @param {double &} current_time; current time
     * @param {const double &} time_step_width; period of time during which we perform integration
     */
	virtual void operator()(
        flight_zone &fz,
        aircraft &ac,
        pilot &pl,
        double &current_time,
        const double &time_step_width) = 0;
};

}

#endif
