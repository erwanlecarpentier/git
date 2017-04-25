#include <L2Fsim/simulation.hpp>

#include <L2Fsim/aircraft/beeler_glider/beeler_glider.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_state.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_command.hpp>

#include <L2Fsim/pilot/passive_pilot.hpp>
#include <L2Fsim/pilot/q_learning/q_learning_pilot.hpp>
#include <L2Fsim/pilot/mcts/b03_uct_pilot.hpp>

#include <L2Fsim/flight_zone/flat_thermal_soaring_zone.hpp>
#include <L2Fsim/flight_zone/flat_zone.hpp>

#include <L2Fsim/stepper/euler_integrator.hpp>
#include <L2Fsim/stepper/rk4_integrator.hpp>

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <libconfig.h++>

using namespace L2Fsim;

int main() {
    srand(time(NULL));
    double t_lim, Dt, nb_dt, t=0.;

	libconfig::Config cfg;
    cfg.readFile("demo/config.cfg");
    if(cfg.lookupValue("limit_time", t_lim)
    && cfg.lookupValue("time_step_width", Dt)
    && cfg.lookupValue("nb_sub_time_step", nb_dt)) {
        t_lim = cfg.lookup("limit_time");
        Dt = cfg.lookup("time_step_width");
        nb_dt = cfg.lookup("nb_sub_time_step");
    }
    else {
        std::cout << "Error: variable initialisation in config file" << std::endl;
    }

    /** Environment */
    double windx=0., windy=0.;
    // read config1.txt
    //flat_zone my_zone(windx,windy);
    flat_thermal_soaring_zone my_zone("data/config1.txt");

    /** Initial state */
	double x0 = -500.; // m
	double y0 = 0.; // m
	double z0 = 1000.; // m
	double V0 = 20.; // m/s
	double gamma0 = 0.; // rad
	double khi0 = 0.; // rad
	double alpha0 = 0.; // rad
	double beta0 = 0.; // rad
	double sigma0 = 0.; // rad
	//TODO put the following into the constructor
	double xdot0 = V0 * cos(khi0) + cos(gamma0);
	double ydot0 = V0 * sin(khi0) + cos(gamma0);
	double zdot0 = V0 * sin(gamma0);
	double Vdot0 = 0.;
	double gammadot0 = 0.;
	double khidot0 = 0.;
    beeler_glider_state my_state(x0,y0,z0,V0,gamma0,khi0,alpha0,beta0,sigma0,xdot0,ydot0,zdot0,Vdot0,gammadot0,khidot0,t);

    /** Initial command */
    beeler_glider_command my_command;

    /** Aircraft */
	beeler_glider my_glider(my_state,my_command);
	//my_glider.update_state_dynamic(my_zone, 0., my_glider.get_state());

    /** Stepper */
	euler_integrator my_stepper(Dt/nb_dt);
	//rk4_integrator my_stepper(dt);

    /**
     * Pilot
     * @param {double} angle_rate_magnitude; rate at which the pilot can modify the angles
     * @param {double} ep; epsilon for Q-learning
     * @param {double} lr; 'learning rate' for Q-learning
     * @param {double} df; discount factor for Q-learning
     */
    double angle_rate_magnitude = 1e-2;//3*1e-1 * 3.14 / 180.;
    double ep=1e-2, lr=1e-3;
    double df=.9;
    double uct_parameter = 1./sqrt(2.);//=0.70710678118
    double uct_tsw=Dt, uct_stsw=uct_tsw;
    unsigned int horizon=500, computational_budget=1e3;
    passive_pilot my_pilot(angle_rate_magnitude);
    //q_learning_pilot my_pilot(angle_rate_magnitude,ep,lr,df);
    /*b03_uct_pilot my_pilot(
        my_stepper.transition_function,
        my_glider,
        my_zone,
        angle_rate_magnitude,
        uct_parameter,
        uct_tsw,
        uct_stsw,
        df,
        horizon,
        computational_budget);*/

    /** Initialize the simulation */
    simulation mysim;
	mysim.fz = &my_zone;
	mysim.ac = &my_glider;
	mysim.st = &my_stepper;
	mysim.pl = &my_pilot;

	/** Run the simulation */
    while(t<t_lim) {
        std::cout << "t = " << t << std::endl;
		mysim.step(t,Dt);
	}
}
