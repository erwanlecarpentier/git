#ifndef L2FSIM_EULER_INTEGRATOR_HPP_
#define L2FSIM_EULER_INTEGRATOR_HPP_

#include <stepper.hpp>

/**
 * @brief Euler stepper
 *
 * @file euler_integrator.hpp
 * @version 1.1
 * @since 1.0
 */

namespace L2Fsim {

class euler_integrator : public stepper {
public:
    /**
     * @brief Attributes
     *
     * @param {double} dt; width of the integration sub-step
     */
    double dt;

	/**
	 * @brief Constructor
	 */
	euler_integrator(double _dt=.01) : dt(_dt) {}

    /**
     * @brief Transition function
     *
     * Perform a transition given: an aircraft model with a correct state and command; an atmospheric model; the current time; the time-step-width and the sub-time-step-width.
     * @note static method for use within an external simulator
     * @param {aircraft &} ac; aircraft model
     * @param {flight_zone &} fz; atmosphere model
     * @param {double &} current_time; current time
     * @param {const double} time_step_width; time-step-width
     * @param {const double} sdt; sub-time-step-width
     */
    static void transition_function(
        aircraft &ac,
        flight_zone &fz,
        double &current_time,
        const double time_step_width,
        const double sdt)
    {
        unsigned int niter = (int)(time_step_width/sdt);
        for(unsigned int n=0; n<niter; ++n) {
            ac.apply_command();
            //// EULER UPDATE
            ac.update_state_dynamic(fz,current_time,ac.get_state());
            ac.get_state().apply_dynamic(sdt);
            //// END EULER UPDATE
            current_time += sdt;
            ac.get_state().set_time(current_time);
        }
    }

    /**
     * @brief Stepping operator
     *
     * @param {flight_zone &} fz; flight zone
     * @param {aircraft &} ac; aircraft
     * @param {pilot &} pl; pilot
     * @param {double &} current_time; current time
     * @param {const double} time_step_width; period of time during which we perform integration
     * @param {bool &} eos; end of simulation, the simulation reached the bounds of its model and must be stopped (e.g. limit of aircraft model validity)
     */
    void operator()(
        flight_zone &fz,
        aircraft &ac,
        pilot &pl,
        double &current_time,
        const double time_step_width,
        bool &eos) override
    {
        // 1. Apply the policy and store the command into command attribute of aircraft
        if (!fz.is_within_fz(ac.get_state().getx(),ac.get_state().gety(),ac.get_state().getz())) {
            pl.out_of_boundaries(ac.get_state(),ac.get_command());
        } else {
            pl(ac.get_state(),ac.get_command());
        }

        // 2. Apply the transition with Euler method
        transition_function(ac,fz,current_time,time_step_width,dt);

        // 3. Check aircraft's configuration validity
        if(!ac.is_in_model()){eos=true;}
    }
};

}

#endif // L2FSIM_EULER_INTEGRATOR_HPP_
