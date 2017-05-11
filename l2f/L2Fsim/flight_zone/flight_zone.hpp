#ifndef L2FSIM_FLIGHT_ZONE_HPP_
#define L2FSIM_FLIGHT_ZONE_HPP_

#include <vector>

/**
 * @file flight_zone.hpp
 * @version 1.0
 *
 * @brief The abstract class flight_zone implementing the atmospheric environment into which the aircraft moves
 * A flight zone holds two important concepts:
 * - it has a characterization of the wind w in the flight zone at a given time;
 * - it has an altitude z of the ground surface at all points in the flight zone;
 */

namespace L2Fsim {

class flight_zone {

public:
    virtual ~flight_zone() = default;

    /**
	 * @brief Compute the wind velocity vector w at coordinate (x,y,z,t)
	 * @param {double} x, y, z, t; coordinates in earth frame
	 * @param {std::vector<double> &} w; wind velocity vector [wx, wy, wz]
	 */
	virtual flight_zone& wind(double x, double y, double z, double t, std::vector<double> &w) =0;

    /**
	 * @brief Compute the altitude at (x,y)
	 * @param {const double &} x, y; coordinates in earth frame
	 * @param {double &} z; altitude
	 */
    flight_zone& ground(const double &x, const double &y, double &z)
    {
        (void) x; (void) y; z=0.;// this is default
        return *this;
    }
};

}

#endif
