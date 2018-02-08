#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <libconfig.h++>
#include <src/simulation.hpp>
#include <src/utils/cfg_reader.hpp>

using namespace L2Fsim;

/**
 * @brief Create environment
 *
 * Create an environment and save it at the specified location.
 * @param {bool} save; if true, save the scenario for vizualization
 * @param {double} dt; thermal refreshment rate (s)
 * @param {int} model; thermal model selection
 * 1: Allen
 * 2: Childress
 * 3: Lenschow
 * 4: Geodon
 * 5: Lawrance
 */
void create_environment(bool save, double dt=1., int model=1) {
    double t_start = -500.;
    double t_limit = 1000.;
    double windx = 0.;
    double windy = 0.;
    double w_star_min = 2.;
    double w_star_max = 2.8;
    double zi_min = 1300.;
    double zi_max = 1400.;
    double lifespan_min = 600.;
    double lifespan_max = 1200.;
    double x_min = -1500.; ///< Range
    double x_max = +1500.; ///< Range
    double y_min = -1500.; ///< Range
    double y_max = +1500.; ///< Range
    double z_min = +0.; ///< Range
    double z_max = +2000.; ///< Range
    double ksi_min = .3;
    double ksi_max = .7;
    double d_min = 150.;
    unsigned int nbth = 15; ///< Number of thermals

    // 1. Initialize an empty zone and define its boundaries
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
    //flat_thermal_soaring_zone fz_from_file("config/fz_scenario.csv","config/fz_config.csv");

    // 2. Create a scenario i.e. create the thermals
    fz.create_scenario(dt,model);
    //fz.print_scenario(); // print the created thermals (optional)

    // 3.Save data for visualization (optional)
    if(save) {
        double dx = 5., dy = 5.; // mesh precision
        std::vector<double> z_vec = {10.};
        std::vector<double> t_vec = {0., 50.};
        fz.save_updraft_values(dx,dy,z_vec,t_vec,"data/updraft_field.dat");
    }

    // 4. Save the scenario
    fz.save_scenario("config/fz_scenario.csv");
    fz.save_fz_cfg("config/fz_config.csv");
}

/**
 * @brief Single run
 *
 * Run a single simulation using a configuration file.
 * @param {const char *} cfg_path; path to the configuration file
 */
void run_with_config(const char *cfg_path) {
    cfg_reader cfgr;
    libconfig::Config cfg;
    cfg.readFile(cfg_path);

    // 1. Initialize the simulation
    simulation mysim;

    // 2. General settings
	mysim.st_log_path = cfgr.read_st_log_path(cfg);
	mysim.fz_log_path = cfgr.read_fz_log_path(cfg);
    double Dt = .1, t_lim = 1e3, nb_dt = 1., t = 0.; // default values
    cfgr.read_time_variables(cfg,t_lim,Dt,nb_dt);

    // 3. Environment
    mysim.fz = cfgr.read_environment(cfg);

    // 4. Aircraft
	mysim.ac = cfgr.read_aircraft(cfg);

    // 5. Stepper
	mysim.st = cfgr.read_stepper(cfg,Dt/nb_dt);

    // 6. Pilot
	mysim.pl = cfgr.read_pilot(cfg);

	// 7. Run the simulation
	bool eos = false;
	mysim.clear_saves();
    while(!(is_greater_than(t,t_lim)) && !eos) {
        std::cout << t << std::endl;
        mysim.save();
        mysim.step(t,Dt,eos);
    }

	// 8. End of simulation
	std::cout << "End of simulation\n";
}

int main() {
    try {
        srand(time(NULL));
        create_environment(false);
        run_with_config("config/main.cfg");
    }
    catch(const std::exception &e) {
        std::cerr<<"[error] In main(): standard exception caught: "<<e.what()<<std::endl;
    }
    catch(...) {
        std::cerr<<"[error] In main(): unknown exception caught"<<std::endl;
    }
    return 0;
}
