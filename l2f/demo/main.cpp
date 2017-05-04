#include <L2Fsim/simulation.hpp>

#include <L2Fsim/aircraft/beeler_glider/beeler_glider.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_state.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_command.hpp>

#include <L2Fsim/pilot/passive_pilot.hpp>
#include <L2Fsim/pilot/heuristic_pilot.hpp>
#include <L2Fsim/pilot/q_learning/q_learning_pilot.hpp>
#include <L2Fsim/pilot/mcts/b03_uct_pilot.hpp>

#include <L2Fsim/flight_zone/flat_thermal_soaring_zone.hpp>
#include <L2Fsim/flight_zone/flat_zone.hpp>

#include <L2Fsim/stepper/euler_integrator.hpp>
#include <L2Fsim/stepper/rk4_integrator.hpp>

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <libconfig.h++>

#define TO_RAD 0.01745329251

using namespace L2Fsim;

void read_time_variables(const libconfig::Config &cfg, double &t_lim, double &Dt, double &nb_dt) {
    if(cfg.exists("limit_time")
    && cfg.exists("time_step_width")
    && cfg.exists("nb_sub_time_step")) {
        t_lim = cfg.lookup("limit_time");
        Dt = cfg.lookup("time_step_width");
        nb_dt = cfg.lookup("nb_sub_time_step");
    }
    else {
        std::cout << "Error: variable initialisation in config file" << std::endl;
    }
}

flight_zone * read_environment_variables(const libconfig::Config &cfg) {
    if(cfg.exists("envt_selector")
    && cfg.exists("wx")
    && cfg.exists("wy")
    && cfg.exists("envt_path")) {
        unsigned int sl = cfg.lookup("envt_selector");
        switch(sl) {
        case 0: { // flat_zone
            double wx = cfg.lookup("wx");
            double wy = cfg.lookup("wy");
            return new flat_zone(wx,wy);
        }
        case 1: { // flat_thermal_soaring_zone
            std::string path = cfg.lookup("envt_path");
            return new flat_thermal_soaring_zone(path);
        }
        }
    }
    else {
        std::cout << "Error: variable initialisation in config file" << std::endl;
    }
    return NULL;
}

void read_state_variables(
    const libconfig::Config &cfg,
    double &_x0,
    double &_y0,
    double &_z0,
    double &_V0,
    double &_gamma0,
    double &_khi0,
    double &_alpha0,
    double &_beta0,
    double &_sigma0)
{
    if(cfg.lookupValue("x0", _x0)
    && cfg.lookupValue("y0", _y0)
    && cfg.lookupValue("z0", _z0)
    && cfg.lookupValue("V0", _V0)
    && cfg.lookupValue("gamma0", _gamma0)
    && cfg.lookupValue("khi0", _khi0)
    && cfg.lookupValue("alpha0", _alpha0)
    && cfg.lookupValue("beta0", _beta0)
    && cfg.lookupValue("sigma0", _sigma0)) {
        _x0 = cfg.lookup("x0");
        _y0 = cfg.lookup("y0");
        _z0 = cfg.lookup("z0");
        _V0 = cfg.lookup("V0");
        _gamma0 = cfg.lookup("gamma0"); _gamma0 *= TO_RAD;
        _khi0 = cfg.lookup("khi0"); _khi0 *= TO_RAD;
        _alpha0 = cfg.lookup("alpha0"); _alpha0 *= TO_RAD;
        _beta0 = cfg.lookup("beta0"); _beta0 *= TO_RAD;
        _sigma0 = cfg.lookup("sigma0"); _sigma0 *= TO_RAD;
    }
    else {
        std::cout << "Error: variable initialisation in config file" << std::endl;
    }
}

stepper * read_stepper_variable(const libconfig::Config &cfg, const double &sub_dt) {
    if(cfg.exists("stepper_selector")) {
        unsigned int sl = cfg.lookup("stepper_selector");
        switch(sl) {
        case 0: { // euler_integrator
            return new euler_integrator(sub_dt);
        }
        case 1: { // rk4_integrator
            return new rk4_integrator(sub_dt);
        }
        }
    }
    else {
        std::cout << "Error: variable initialisation in config file" << std::endl;
    }
    return NULL;
}

void run_with_config(const char *cfg_path) {
    libconfig::Config cfg;
    cfg.readFile(cfg_path);

    srand(time(NULL));
    double t_lim, Dt, nb_dt, t=0.;
    read_time_variables(cfg,t_lim,Dt,nb_dt);

    /** Environment */
    flight_zone *my_zone = read_environment_variables(cfg);

    /** Initial state */
	double x0, y0, z0, V0, gamma0, khi0, alpha0, beta0, sigma0;
	read_state_variables(cfg,x0,y0,z0,V0,gamma0,khi0,alpha0,beta0,sigma0);
    beeler_glider_state my_state(x0,y0,z0,V0,gamma0,khi0,alpha0,beta0,sigma0);

    /** Initial command */
    beeler_glider_command my_command;

    /** Aircraft */
	beeler_glider my_glider(my_state,my_command);

    /** Stepper */
	stepper *my_stepper = read_stepper_variable(cfg,Dt/nb_dt);

    /** Pilot */
    double angle_rate_magnitude = 1e-2;
    //double ep=1e-2, lr=1e-3;
    //double df=.9;
    //double uct_parameter = 1./sqrt(2.);
    //double uct_tsw=2., uct_stsw=uct_tsw/1.;
    //unsigned int horizon=100, computational_budget=10000;
    passive_pilot my_pilot(angle_rate_magnitude);
    //heuristic_pilot my_pilot(angle_rate_magnitude);
    //q_learning_pilot my_pilot(angle_rate_magnitude,ep,lr,df);
    /*b03_uct_pilot my_pilot(
        my_stepper.transition_function, // todo: create a new stepper (euler for simplicity)
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
	mysim.fz = my_zone;
	mysim.ac = &my_glider;
	mysim.st = my_stepper;
	mysim.pl = &my_pilot;

	/** Run the simulation */
    while(t<t_lim) {
        //std::cout << "t = " << t << std::endl;
		mysim.step(t,Dt);
	}

	/** Delete the dynamically created variables */
	delete my_zone;
	delete my_stepper;
	std::cout << "Program run successfully" << std::endl;
}

int main() {
    run_with_config("demo/config.cfg");
    return 0;
}
