#ifndef L2FSIM_FLIGHT_ZONE_HPP_
#define L2FSIM_FLIGHT_ZONE_HPP_

#include <vector>

/**
 * @file flight_zone.hpp
 * @version 1.0
 * @brief The abstract class flight_zone implementing the atmospheric environment into which the aircraft moves
 * A flight zone holds two important concepts:
 * - it has a characterization of the wind w in the flight zone at a given time;
 * - it has an altitude z of the ground surface at all points in the flight zone;
 */

namespace L2Fsim {

class flight_zone {

public:
    /** @brief Default destructor */
    virtual ~flight_zone() = default;

    /**
	 * @brief Compute the wind velocity vector w at coordinate (x,y,z,t)
	 * @param {double} x, y, z, t; coordinates in earth frame
	 * @param {std::vector<double> &} w; wind velocity vector [wx, wy, wz]
	 */
	virtual flight_zone& wind(double x, double y, double z, double t, std::vector<double> &w) = 0;

    /**
	 * @brief Compute the altitude at (x,y)
	 * @param {double} x, y; coordinates in earth frame
	 * @param {double &} z; altitude
	 */
    flight_zone& ground(double x, double y, double &z) {
        (void) x; (void) y; z=0.; // this is default
        return *this;
    }

    /**
     * @brief Assert that the aircraft is inside the flight zone
     * @param {double} x, y, z; coordinates  in the earth frame
     * @return true if the input position belongs to the flight zone
     */
    virtual bool is_within_fz(double x, double y, double z) = 0;
};

}

#endif
