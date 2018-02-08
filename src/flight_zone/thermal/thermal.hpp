#ifndef L2FSIM_THERMAL_HPP_
#define L2FSIM_THERMAL_HPP_

#include <vector>

namespace L2Fsim {

/**
 * @brief Thermal abstract class
 *
 * @file thermal.hpp
 * @version 1.0
 * @brief The abstract class thermal calculates the wind vector w linked with a thermal, at time t
 */

class thermal {
protected:
    /**
     * @brief Attributes
     * @param {double} windx, windy; horizontal wind in the flight zone, responsible for thermal drift
     */
    double windx, windy;

public:
    /**
     * @brief Destructor
     */
    virtual ~thermal() = default;

    /**
     * @brief Get average updraft velocity.
     */
    virtual double get_w_star() = 0;

    /**
     * @brief Get thermal's date of birth.
     */
    virtual double get_t_birth() = 0;

    /**
     * @brief Get thermal lifespan.
     */
    virtual double get_lifespan() = 0;

    /**
     * @brief Get mixing layer thickness.
     */
    virtual double get_zi() = 0;

    /**
     * @brief Get thermal model.
     */
    virtual int get_model() = 0;

    /**
     * @brief Get shape parameter.
     */
    virtual double get_ksi() = 0;

    /**
     * @brief Get vector of thermal centers.
     */
    virtual std::vector<double> get_center() = 0;

    /**
     * @brief Is thermal alive
     *
     * Test if the thermal is alive.
     * @param {double } t; current time
     * @return Return true if the thermal is alive.
     */
    virtual bool is_alive(const double t) = 0;

    /**
     * @brief Lifetime coefficient
     *
     * @param {const double} t; current time
     * @return Return the thermal life cycle coefficient.
     */
    virtual double lifetime_coefficient(const double t) = 0;

    /**
     * @brief Distance to updraft center
     *
     * Get the Euclidean distance between a point (x,y,z) and the center of the thermal.
     * If z =! 0 then it considers the drift of the thermal center at the height z.
     * @param {const double} x, y, z; coordinate in the earth frame
     * @return Return the distance to the center.
	 */
    virtual double dist_to_updraft_center(const double x, const double y, const double z) = 0;

    /**
     * @brief Set horizontal wind
     *
     * Set the wind speed onto the x-axis and y-axis.
     * @param {const double} wx, wy; wind velocity vector coordinates in the earth frame
	 */
    void set_horizontal_wind(const double wx, const double wy) {
        windx = wx;
        windy = wy;
    }

	/**
     * @brief Wind
     *
     * Computes the wind vector w, at point (x,y,z), at time t.
     * @param {double} x, y, z; coordinate in the earth frame
     * @param {double} t; time
     * @param {std::vector<double> &} w; wind velocity vector in the earth frame
	 */
	virtual thermal& wind(const double x, const double y, const double z, const double t, std::vector<double> &w) = 0;

	/**
	 * @brief Print
	 *
	 * Print the thermal's features in the standard output stream.
	 */
	virtual void print() = 0;
};

}

#endif // L2FSIM_THERMAL_HPP_
