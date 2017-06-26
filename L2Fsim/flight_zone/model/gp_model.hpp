#ifndef L2FSIM_GP_MODEL_HPP_
#define L2FSIM_GP_MODEL_HPP_

//#include <L2Fsim/flight_zone/flight_zone.hpp>
//#include <L2Fsim/flight_zone/flat_zone.hpp>
//#include <L2Fsim/flight_zone/thermal/std_thermal.hpp>
//#include <string>
//#include <fstream>
//#include <sstream>
//#include <chrono>

#include <L2Fsim/utils/gaussian_process.hpp>

/** @file gp_model.hpp
 * @version 1.0
 * @brief GP model for the flight zone based on 'flat_thermal_soaring_zone.hpp' flight zones
 */

namespace L2Fsim {

class gp_model : public flat_zone {
    /** @brief Attributes
     * @param {flat_thermal_soaring_zone} fz; flight zone form which data points are extracted
     */
    flat_thermal_soaring_zone fz;

    /** @brief Constructor */
    gp_model(flat_thermal_soaring_zone _fz) : fz(_fz) {}

public :
	/** @brief Compute the wind velocity vector w at coordinate (x, y, z, t)
     * @param {double} x, y, z, t; coordinates
     * @param {std::vector<double> &} w; wind velocity vector (wx, wy, wz)
     */
    gp_model& wind(double x, double y, double z, double t, std::vector<double> &w) {
        //TODO
        (void) x;
        (void) y;
        (void) z;
        (void) t;
        (void) w;
        return *this;
    }
};

}

#endif
