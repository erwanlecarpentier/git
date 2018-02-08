#ifndef L2FSIM_RK4_INTEGRATOR_HPP_
#define L2FSIM_RK4_INTEGRATOR_HPP_

#include <stepper.hpp>

/**
 * @brief RK4 stepper
 *
 * @file rk4_integrator.hpp
 * @version 1.1
 * @since 1.0
 */

namespace L2Fsim {

class rk4_integrator : public stepper {
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
	rk4_integrator(double _dt=.01) : dt(_dt) {}

    /**
     * @brief Transition function
     *
     * Perform a transition given: an aircraft model with a correct state and command; an atmospheric model; the current time; the time-step-width and the sub-time-step-width.
     * Static method for use within an external simulator
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
            //// RK4 UPDATE
            ac.get_state().clear_dynamic(); // s: [ x , 0 ]

            state *s1 = ac.get_state().duplicate(); // s1: [ x1 , 0 ] with x1=x static variables at time t
            ac.update_state_dynamic(fz,current_time,*s1); // s1: [ x1 , k1 ] with k1=f(t,x1)

            state *s2 = s1->duplicate(); // s2: [ x1, k1 ]
            s2->apply_dynamic(.5*sdt); // s2: [ x2 , k1 ] with x2=s1+.5*sdt*k1
            ac.update_state_dynamic(fz,current_time+.5*sdt,*s2); // s2: [ x2 , k2 ] with k2=f(t+.5*sdt,x(t)+.5*sdt*k1)

            state *s3 = s1->duplicate(); // s3: [ x1 , k1 ]
            s3->set_dynamic(*s2); // s3: [ x1 , k2 ]
            s3->apply_dynamic(.5*sdt); // s3: [ x3 , k2 ] with x3=x1+.5*sdt*k2
            ac.update_state_dynamic(fz,current_time+.5*sdt,*s3); // s3: [ x3 , k3 ] with k3=f(t+.5*sdt,x(t)+.5*sdt*k2)

            state *s4 = s1->duplicate(); // s4: [ x1 , k1 ]
            s4->set_dynamic(*s3); // s4: [ x1 , k3 ]
            s4->apply_dynamic(sdt); // s4: [ x4 , k3 ] with x4=x1+sdt*k3
            ac.update_state_dynamic(fz,current_time+sdt,*s4); // s4: [ x4 , k4 ] with k4=f(t+sdt,x(t)+sdt*k3)

            ac.get_state().add_to_dynamic(*s1,1.); // s: [ x , k1 ]
            ac.get_state().add_to_dynamic(*s2,2.); // s: [ x , k1 + 2*k2 ]
            ac.get_state().add_to_dynamic(*s3,2.); // s: [ x , k1 + 2*k2 + 2*k3 ]
            ac.get_state().add_to_dynamic(*s4,1.); // s: [ x , xdot ] with wdot = k1 + 2*k2 + 2*k3 + k4
            ac.get_state().apply_dynamic(sdt/6.); // s: [ x + xdot/6 , xdot ]

            delete s4;
            delete s3;
            delete s2;
            delete s1;
            //// END RK4 UPDATE
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

        // 2. Apply the transition with RK4 method
        transition_function(ac,fz,current_time,time_step_width,dt);

        // 3. Check aircraft's configuration validity
        if(!ac.is_in_model()){eos=true;}
    }
};

}

#endif // L2FSIM_RK4_INTEGRATOR_HPP_
