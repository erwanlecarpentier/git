#include <L2Fsim/flight_zone/flat_thermal_soaring_zone.hpp>
#include <iostream>

using namespace L2Fsim;

/*
Questions :
- Formule de nbMaxThermals = floor(0.6*(x_max-x_min)*(y_max-y_min)/(d_min*zi_avg))
*/

int main() {
    srand(time(NULL));

    /**
     * Parameters
     * @note See the used flight zone for description of the parameters
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
    double w_star_min = 2.;
    double w_star_max = 5.;
    double zi_min = 1300.;
    double zi_max = 1400.;
    double lifespan_min = 300.;
    double lifespan_max = 600.;
    double x_min = -1000.;
    double x_max = +1000.;
    double y_min = -1000.;
    double y_max = +1000.;
    double z_min = +0.;
    double z_max = +2000.;
    double ksi_min = .3;
    double ksi_max = .7;
    double d_min = 200.;

    /// 1. Initialize an empty zone and define its limitations
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
        d_min);
    //flat_thermal_soaring_zone fz_from_file("config/thermal_scenario.csv");

    /// 2. Create a scenario i.e. create the thermals
    fz.create_scenario(dt,model);
    fz.print_scenario();

    /// 3 : Save data for visualization
    double dx = 50.; // mesh precision in x direction
    double dy = dx; // mesh precision in y direction
    double z = 500.;  // altitude of the saved updraft field
    double t = 500.;  // time of the saved updraft field
    fz.save_updraft_values(dx,dy,z,t,"data/updraft_field.dat");

    /// 4. Save the scenario
    fz.save_scenario("config/thermal_scenario.csv");

    return 0;
}
