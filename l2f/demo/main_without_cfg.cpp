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

#define TO_RAD 0.01745329251

using namespace L2Fsim;

void run() {
    /** Time variables */
    double Dt=.1, t_lim=1e3, nb_dt=1., t=0.; // default values

    /** Environment */
    flat_thermal_soaring_zone my_zone("config/thermal_scenario.csv",0.);
    //flat_zone my_zone(0.,0.);

    /** Initial state */
	double x0=0., y0=0., z0=500., V0=14.;
	double gamma0=-3.*TO_RAD, khi0=0.*TO_RAD;
	double alpha0=0.*TO_RAD, beta0=0.*TO_RAD, sigma0=0.*TO_RAD;
	double maximum_angle_magnitude = 30.*TO_RAD;
    beeler_glider_state my_state(x0,y0,z0,V0,gamma0,khi0,alpha0,beta0,sigma0,maximum_angle_magnitude);

    /** Initial command */
    beeler_glider_command my_command;

    /** Aircraft */
	beeler_glider my_glider(my_state,my_command);

    /** Stepper */
	euler_integrator my_stepper(Dt/nb_dt);

    /** Pilot */
    double angle_rate_magnitude = .1;
    passive_pilot my_pilot(angle_rate_magnitude);
    //heuristic_pilot my_pilot(angle_rate_magnitude);

    /** Initialize the simulation */
    simulation mysim;
	mysim.fz = &my_zone;
	mysim.ac = &my_glider;
	mysim.st = &my_stepper;
	mysim.pl = &my_pilot;

	/** Run the simulation */
	bool eos = false;
    while(t<=t_lim && !eos) {
        std::cout<<"t = "<<t<<std::endl;
        mysim.step(t,Dt,eos);
    }

	std::cout << "Program run successfully" << std::endl;
}

int main() {
    srand(time(NULL));
    run();
    return 0;
}
