#ifndef L2FSIM_RK4_INTEGRATOR_HPP_
#define L2FSIM_RK4_INTEGRATOR_HPP_

#include <L2Fsim/stepper/stepper.hpp>
#include <L2Fsim/utils/utils.hpp>
#include <cstdio>
#include <cstdlib>

/**
 * @file rk4_integrator.hpp
 * @brief RK4 stepper
 * @version 1.1
 * @since 1.0
 */

namespace L2Fsim {

class rk4_integrator : public stepper {
public:
    /**
     * @brief Attributes
     * @param {double} dt; width of the integration sub-step
     * @param {std::string} state_log_filename; file name for log
     */
    double dt;
    std::string state_log_path = "data/state.dat";
    std::string wind_log_path = "data/wind.dat";

	/** @brief Constructor */
	rk4_integrator(double _dt=.001) : dt(_dt) {}

    /**
     * @brief Transition function; perform a transition given: an aircraft model with a correct state and command; an atmospheric model; the current time; the time-step-width and the sub-time-step-width
     * @note static method for use within an external simulator
     * @param {aircraft &} ac; aircraft model
     * @param {flight_zone &} fz; atmosphere model
     * @param {double &} current_time; current time
     * @param {const double &} time_step_width; time-step-width
     * @param {const double &} dt; sub-time-step-width
     */
    static void transition_function(
        aircraft &ac,
        flight_zone &fz,
        double &current_time,
        const double &time_step_width,
        const double &dt)
    {
        for(int n=0; n<time_step_width/dt; ++n) {
            ac.apply_command();
            //// RK4 UPDATE
            ac.get_state().clear_dynamic();

            state *s1 = ac.get_state().duplicate(); // s1: [ x1 , 0 ] with x1=x(t) static variables at time t
            ac.update_state_dynamic(fz,current_time,*s1); // s1: [ x1 , k1 ] with k1=f(t,x1)

            state *s2 = s1->duplicate(); // s2: [ x1, k1 ]
            s2->apply_dynamic(.5*dt); // s2: [ x2 , k1 ] with x2=s1+.5*dt*k1
            ac.update_state_dynamic(fz,current_time+.5*dt,*s2); // s2: [ x2 , k2 ] with k2=f(t+.5*dt,x(t)+.5*dt*k1)

            state *s3 = s1->duplicate(); // s3: [ x1 , k1 ]
            s3->set_dynamic(*s2); // s3: [ x1 , k2 ]
            s3->apply_dynamic(.5*dt); // s3: [ x3 , k2 ] with x3=x1+.5*dt*k2
            ac.update_state_dynamic(fz,current_time+.5*dt,*s3); // s3: [ x3 , k3 ] with k3=f(t+.5*dt,x(t)+.5*dt*k2)

            state *s4 = s1->duplicate(); // s4: [ x1 , k1 ]
            s4->set_dynamic(*s3); // s4: [ x1 , k3 ]
            s4->apply_dynamic(dt); // s4: [ x4 , k3 ] with x4=x1+dt*k3
            ac.update_state_dynamic(fz,current_time+dt,*s4); // s4: [ x4 , k4 ] with k4=f(t+dt,x(t)+dt*k3)

            ac.get_state().add_to_dynamic(*s1,1.);
            ac.get_state().add_to_dynamic(*s2,2.);
            ac.get_state().add_to_dynamic(*s3,2.);
            ac.get_state().add_to_dynamic(*s4,1.);
            ac.get_state().apply_dynamic(dt/6.);

            delete s4;
            delete s3;
            delete s2;
            delete s1;
            //// END RK4 UPDATE
            current_time += dt;
            ac.get_state().update_time(current_time);
        }
    }

    /**
     * Stepping operator
     * @param {flight_zone &} fz; flight zone
     * @param {aircraft &} ac; aircraft
     * @param {pilot &} pl; pilot
     * @param {double &} current_time; current time
     * @param {const double &} time_step_width; period of time during which we perform integration
     */
    void operator()(
        flight_zone &fz,
        aircraft &ac,
        pilot &pl,
        double &current_time,
        const double &time_step_width) override
    {
        /// 1. Apply the policy
        if (ac.get_distance_to_center() > 1200.) {
            pl.out_of_boundaries(ac.get_state(),ac.get_command());
        } else {
            pl(ac.get_state(),ac.get_command());
        }

        /// 2. Save the data
        /// @note output files should be cleared first
        save_vector(ac.get_state().get_save(),state_log_path,std::ofstream::app);
        std::vector<double> w;
        fz.wind(ac.get_state().getx(),
            ac.get_state().gety(),
            ac.get_state().getz(),
            ac.get_state().gett(),w);
        save_vector(w,wind_log_path,std::ofstream::app);

        /// 3. Apply the transition with RK4 method
        transition_function(ac,fz,current_time,time_step_width,dt);

        /// 4. Check aircraft's configuration validity
        ac.is_in_model();
    }
};

}

#endif
