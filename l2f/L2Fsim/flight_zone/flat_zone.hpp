#ifndef L2FSIM_FLAT_ZONE_HPP_
#define L2FSIM_FLAT_ZONE_HPP_

#include <vector>

namespace L2Fsim{


/**
 * The class flat_zone is a derived class of flight_zone
 * It implements a flat ground at sea level (z=0) with a horizontal wind
 * @version 1.1
 * @since 1.0
 */

class flat_zone : public flight_zone {
protected:
    /**
     * Attributes
     * @param {double} windx; global wind velocity along the x-axis of the earth frame
     * @param {double} windy; global wind velocity along the y-axis of the earth frame
     */
	double windx, windy;

public:

    flat_zone(double _windx=0., double _windy=0.) :
        windx(_windx),
        windy(_windy) {}

    /**
	 * Computes the wind vector w, at point (x,y,z), at time t
     * @param {double} x, y, z; coordinates in the earth frame
     * @param {double} t; time
	 * @param {std::vector<double>} w; wind vector windx, windy, windz
	 */
	virtual flat_zone& wind(double x, double y, double z, double t, std::vector<double> &w) override {
        std::vector<double>(3,0.).swap(w);
        w.at(0) = windx;
        w.at(1) = windy;
        return *this;
	}

};

}

#endif
