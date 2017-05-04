#include <L2Fsim/flight_zone/flat_thermal_soaring_zone.hpp>
#include <iostream>

using namespace L2Fsim;

int main() {
    srand(time(NULL));

    /**
     * Configuration
     * @param {double} t_lim;
     * @param {double} deltaT; actualization of thermals
     * @param {int} minX, maxX, minY, maxY, minZ, maxZ; box definition
     * @param {double} wx, wy; horizontal wind vector components in the earth frame
     * @param {double} zi; thermals height
     * @param {double} dmin; radius of a thermal
     * @param {int} model; thermal model selection
     * 1: Allen
     * 2: Childress
     * 3: Lenschow
     * 4: Geodon
     * 5: Lawrance
     */
    double t_lim = 1000.;
    double deltaT = 100.;
    int minX = -1000;
    int maxX = 1000;
    int minY = -1000;
    int maxY = 1000;
    int minZ = 0;
    int maxZ = 2000;
    double wx = 0.;
    double wy = 0.;
    double d_min = 200.;
    double zi = 1400.;
    int model = 1;

    // 1 : create an empty box
    flat_thermal_soaring_zone fz(t_lim,minX,maxX,minY,maxY,minZ,maxZ,wx,wy,d_min,zi);

    // 2 : Simulate a scenario of thermals
    fz.createScenario(deltaT,model);

    // 3 : write data for zslice visualization
    double deltax = 50.; // mesh precision in x direction
    double deltay = deltax; // mesh precision in y direction
    double zslice = 500.;  // height of the windfield
    fz.writeScenario(deltaT,deltax,deltay,zslice,"data/wind_field.dat");

    // 4 : save this configuration
    fz.saveConfig("config/thermal_config.cfg");

    // 5 : call a previous configuration
    //flat_thermal_soaring_zone fz_bis("data/config1.txt");

    return 0;
}
