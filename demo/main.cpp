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

/** @brief Method creating an environment and saving it at the specified location */
void create_environment() {
    /**
     * @brief Parameters; See the used flight zone for description of the parameters
     * @param {double} dt; thermal refreshment rate (s)
     * @param {int} model; thermal model selection
     * 1: Allen
     * 2: Childress
     * 3: Lenschow
     * 4: Geodon
     * 5: Lawrance
     */
    int model = 1;
    double dt = 1.;

    double t_start = -500.;
    double t_limit = 1000.;
    double windx = 0.;
    double windy = 0.;
    double w_star_min = 4.;
    double w_star_max = 6.;
    double zi_min = 1300.;
    double zi_max = 1400.;
    double lifespan_min = 300.;
    double lifespan_max = 600.;
    double x_min = -1500.;
    double x_max = +1500.;
    double y_min = -1500.;
    double y_max = +1500.;
    double z_min = +0.;
    double z_max = +2000.;
    double ksi_min = .3;
    double ksi_max = .7;
    double d_min = 150.;
    unsigned int nbth = 15;

    /// 1. Initialize an empty zone and define its boundaries
    flat_thermal_soaring_zone fz(
        t_start, t_limit,
        windx, windy,
        w_star_min, w_star_max,
        zi_min, zi_max,
        lifespan_min, lifespan_max,
        x_min, x_max,
        y_min, y_max,
        z_min, z_max,
        ksi_min, ksi_max,
        d_min, nbth);
    //flat_thermal_soaring_zone fz_from_file("config/thermal_scenario.csv","config/fz_config.csv");

    /// 2. Create a scenario i.e. create the thermals
    fz.create_scenario(dt,model);
    //fz.print_scenario(); // print the created thermals (optional)

    /// 3 : Save data for visualization (optional)
    /*
    double dx=50., dy=50.; // mesh precision
    double z=500., t=500.;  // altitude and time of the saved updraft field
    fz.save_updraft_values(dx,dy,z,t,"data/updraft_field.dat");
    */

    /// 4. Save the scenario
    fz.save_scenario("config/thermal_scenario.csv");
    fz.save_fz_cfg("config/fz_config.csv");
}

/**
 * @brief Method running a simulation using a configuration file
 * @param {const char *} cfg_path; path to the configuration file
 * @note WORK IN PROGRESS -> config file handling
 */
void run_with_config(const char *cfg_path) {
    cfg_reader cfgr;
    libconfig::Config cfg;
    cfg.readFile(cfg_path);

    /** General settings */
    double Dt=.1, t_lim=1e3, nb_dt=1., t=0.; // default values
    cfgr.read_time_variables(cfg,t_lim,Dt,nb_dt);
    std::string ac_save_path = "data/state.dat";
    std::string fz_save_path = "data/wind.dat";

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
    double angle_rate_magnitude = .05;
    //passive_pilot my_pilot(angle_rate_magnitude);
    //heuristic_pilot my_pilot(angle_rate_magnitude);

    //double ep=1e-2, lr=1e-3, df=.9;
    //q_learning_pilot my_pilot(angle_rate_magnitude,ep,lr,df);*/

    double uct_df=.9;
    double uct_parameter = 1./sqrt(2.);
    double uct_tsw=Dt, uct_stsw=uct_tsw;
    unsigned int uct_horizon=100, uct_budget=1000;
    euler_integrator uct_stepper(uct_stsw);
    flat_thermal_soaring_zone uct_fz("config/thermal_scenario.csv","config/fz_config.csv",0.);
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
	mysim.ac_save_path = ac_save_path;
	mysim.fz_save_path = fz_save_path;

	/** Run the simulation */
	bool eos = false;
	mysim.clear_saves();
    while((t < t_lim || is_equal_to(t,t_lim)) && !eos) {
        std::cout<<t<<std::endl;
        mysim.save();
        mysim.step(t,Dt,eos);
    }

	/** Delete the dynamically created variables */
	delete my_stepper;
	delete my_zone;
	std::cout << "Program run successfully" << std::endl;
}

int main() {
    srand(time(NULL));
    create_environment();
    //run_with_config("config/main_config.cfg");
    return 0;
}
