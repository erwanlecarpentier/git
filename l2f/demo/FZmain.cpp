#include <L2Fsim/flight_zone/flat_thermal_soaring_zone.hpp>
#include <iostream>

using namespace L2Fsim;

/*
Questions :
- Formule de nbMaxThermals = floor(0.6*(x_max-x_min)*(y_max-y_min)/(d_min*zi_avg))
- Formule du lifespan -> < 0. ?
*/

int main() {
    srand(time(NULL));

    /**
     * Parameters
     * @note See the used flight zone for description of the parameters
     * @param {double} dt; actualization of thermals
     * @param {int} model; thermal model selection
     * 1: Allen
     * 2: Childress
     * 3: Lenschow
     * 4: Geodon
     * 5: Lawrance
     */
    int model = 1;
    double dt = 100.; // ???

    double t_start = 0.;
    double t_limit = 1e3;
    double windx = 0.;
    double windy = 0.;
    double w_star_min = 1.;
    double w_star_max = 3.;
    double zi_min = 1300.;
    double zi_max = 1400.;
    double lifespan_min = 300.;
    double lifespan_max = 500.;
    double x_min = -1000.;
    double x_max = +1000.;
    double y_min = -1000.;
    double y_max = +1000.;
    double z_min = +0.;
    double z_max = +2000.;
    double ksi_min = .3;
    double ksi_max = .7;
    double d_min = 200.;

    // 1 : create an empty box
    //flat_thermal_soaring_zone fz(t_lim,minX,maxX,minY,maxY,minZ,maxZ,wx,wy,d_min,zi);
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

    // 2 : Simulate a scenario of thermals
    fz.create_scenario(dt,model);

    // 3 : write data for zslice visualization
    double deltax = 50.; // mesh precision in x direction
    double deltay = deltax; // mesh precision in y direction
    double zslice = 500.;  // height of the windfield
    fz.writeScenario(dt,deltax,deltay,zslice,"data/wind_field.dat");

    // 4 : save this configuration
    fz.save_configuration("config/thermal_config.cfg");

    // 5 : call a previous configuration
    //flat_thermal_soaring_zone fz_bis("data/config1.txt");

    return 0;
}
