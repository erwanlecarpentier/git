#ifndef L2FSIM_RK4_INTEGRATOR_HPP_
#define L2FSIM_RK4_INTEGRATOR_HPP_

#include <L2Fsim/stepper/stepper.hpp>
#include <L2Fsim/utils/utils.hpp>
#include <cstdio>
#include <cstdlib>

namespace L2Fsim {

class rk4_integrator : public stepper {
public:
    /**
     * Attributes
     * @param {double} dt; width of the sub integration step
     * @param {std::string} state_log_filename; file name for log
     */
    double dt;
    std::string state_log_filename = "data/data_plane.txt";

	/** Constructor */
	rk4_integrator(double _dt=.001) : dt(_dt) {}

    /**
     * Euler update operator
     * Modifies the static variables of the state according to the corresponding simulation configuration
     * @warning The dynamic part of the state is set to the value used for the update (cf Euler, RK45, etc.)
     * @param {aircraft &} ac; aircraft
     * @param {flight_zone &} fz; flight zone
     * @param {const double} time; time step
     * @param {const double} dt; integration step
     */
    void rk4_update(
        aircraft &ac,
        flight_zone &fz,
        const double time,
        const double dt)
    {
        ac.get_state().clear_dynamic();

        state *s1 = ac.get_state().duplicate(); // s1: [ x1 , 0 ] with x1=x(t) static variables at time t
        ac.update_state_dynamic(fz,time,*s1); // s1: [ x1 , k1 ] with k1=f(t,x1)

        state *s2 = s1->duplicate(); // s2: [ x1, k1 ]
        s2->apply_dynamic(.5*dt); // s2: [ x2 , k1 ] with x2=s1+.5*dt*k1
        ac.update_state_dynamic(fz,time+.5*dt,*s2); // s2: [ x2 , k2 ] with k2=f(t+.5*dt,x(t)+.5*dt*k1)

        state *s3 = s1->duplicate(); // s3: [ x1 , k1 ]
        s3->set_dynamic(*s2); // s3: [ x1 , k2 ]
        s3->apply_dynamic(.5*dt); // s3: [ x3 , k2 ] with x3=x1+.5*dt*k2
        ac.update_state_dynamic(fz,time+.5*dt,*s3); // s3: [ x3 , k3 ] with k3=f(t+.5*dt,x(t)+.5*dt*k2)

        state *s4 = s1->duplicate(); // s4: [ x1 , k1 ]
        s4->set_dynamic(*s3); // s4: [ x1 , k3 ]
        s4->apply_dynamic(dt); // s4: [ x4 , k3 ] with x4=x1+dt*k3
        ac.update_state_dynamic(fz,time+dt,*s4); // s4: [ x4 , k4 ] with k4=f(t+dt,x(t)+dt*k3)

        ac.get_state().add_to_dynamic(*s1,1.);
        ac.get_state().add_to_dynamic(*s2,2.);
        ac.get_state().add_to_dynamic(*s3,2.);
        ac.get_state().add_to_dynamic(*s4,1.);
        ac.get_state().apply_dynamic(dt/6.);

        delete s4;
        delete s3;
        delete s2;
        delete s1;
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
            pl.out_of_range(ac.get_state(),ac.get_command());
        } else {
            pl(ac.get_state(),ac.get_command());
        }

        /// 2. Save the data
        ac.get_state().save(state_log_filename);
        /*
        std::vector<double> concat;
        concat.reserve( x.size() + u.size() ); // preallocate memory
        concat.insert( concat.end(), x.begin(),x.end() );
        concat.insert( concat.end(), u.begin(), u.end() );
        save_state_into_file(file_name1,concat,current_time);
        save_energy_into_file(file_name2,current_time,time_step_width,obs_old,obs);
        */

        /// 3. Apply the command to the aircraft i.e. modify its state attribute
        ac.apply_command();

        /// 4. Apply the transition with RK4 method
        for(int n=0; n<time_step_width/dt; ++n) {
            rk4_update(ac,fz,current_time,dt);
            current_time += dt;
            ac.get_state().update_time(current_time);
        }

        /// 5. Check aircraft's configuration validity
        ac.is_in_model();
    }
};

}

#endif
