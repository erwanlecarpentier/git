#include <L2Fsim/simulation.hpp>
#include <L2Fsim/cfg_reader.hpp>

#include <L2Fsim/aircraft/beeler_glider/beeler_glider.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_state.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_command.hpp>

#include <L2Fsim/flight_zone/flat_thermal_soaring_zone.hpp>
#include <L2Fsim/flight_zone/flat_zone.hpp>

#include <L2Fsim/pilot/passive_pilot.hpp>
#include <L2Fsim/pilot/heuristic_pilot.hpp>
#include <L2Fsim/pilot/q_learning/q_learning_pilot.hpp>
#include <L2Fsim/pilot/mcts/b03_uct_pilot.hpp>

#include <L2Fsim/stepper/euler_integrator.hpp>
#include <L2Fsim/stepper/rk4_integrator.hpp>

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <libconfig.h++>

using namespace L2Fsim;

void run_with_config(const char *cfg_path) {
    cfg_reader cfgr;
    libconfig::Config cfg;
    cfg.readFile(cfg_path);

    double Dt=.1, t_lim=1e3, nb_dt=1., t=0.; // default values
    cfgr.read_time_variables(cfg,t_lim,Dt,nb_dt);

    /** Environment */
    flight_zone *my_zone = cfgr.read_environment_variables(cfg);

    /** Initial state */
	double x0, y0, z0, V0, gamma0, khi0, alpha0, beta0, sigma0, mam;
	cfgr.read_state_variables(cfg,x0,y0,z0,V0,gamma0,khi0,alpha0,beta0,sigma0,mam);
    beeler_glider_state my_state(x0,y0,z0,V0,gamma0,khi0,alpha0,beta0,sigma0,mam);

    /** Initial command */
    beeler_glider_command my_command;

    /** Aircraft */
	beeler_glider my_glider(my_state,my_command);

    /** Stepper */
	stepper *my_stepper = cfgr.read_stepper_variable(cfg,Dt/nb_dt);

    /** Pilot */
    double angle_rate_magnitude = 1e-1;
    //passive_pilot my_pilot(angle_rate_magnitude);
    //heuristic_pilot my_pilot(angle_rate_magnitude);

    //double ep=1e-2, lr=1e-3, df=.9;
    //q_learning_pilot my_pilot(angle_rate_magnitude,ep,lr,df);*/

    double uct_df=.9;
    double uct_parameter = 1./sqrt(2.);
    double uct_tsw=1., uct_stsw=uct_tsw/1.;
    unsigned int uct_horizon=100, uct_budget=1000;
    euler_integrator uct_stepper(uct_stsw);
    flat_thermal_soaring_zone uct_fz("config/thermal_scenario.csv",0.);
    b03_uct_pilot my_pilot(
        my_glider,
        uct_fz,
        uct_stepper.transition_function,
        angle_rate_magnitude,
        uct_parameter,
        uct_tsw,
        uct_stsw,
        uct_df,
        uct_horizon,
        uct_budget);

    /** Initialize the simulation */
    simulation mysim;
	mysim.fz = my_zone;
	mysim.ac = &my_glider;
	mysim.st = my_stepper;
	mysim.pl = &my_pilot;

	/** Run the simulation */
	bool eos = false;
    while(t<t_lim) {
        std::cout<<"t = "<<t<<std::endl;
        mysim.step(t,Dt,eos);
    }

	/** Delete the dynamically created variables */
	delete my_stepper;
	delete my_zone;
	std::cout << "Program run successfully" << std::endl;
}

int main() {
    srand(time(NULL));
    run_with_config("config/main_config.cfg");
    return 0;
}
