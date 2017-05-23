#ifndef L2FSIM_THERMAL_HPP_
#define L2FSIM_THERMAL_HPP_

#include <vector>

/**
 * @file thermal.hpp
 * @version 1.0
 * @brief The abstract class thermal calculates the wind vector w linked with a thermal, at time t
 */

namespace L2Fsim {

class thermal {
protected:
    /**
     * @brief Attributes
     * @param {double} windx, windy; horizontal wind in the flight zone, responsible for thermal drift
     */
    double windx, windy;

public:
    /** Destructor */
    virtual ~thermal() = default;

    /** Get average updraft velocity */
    virtual double get_w_star() = 0;

    /** Get thermal's date of birth */
    virtual double get_t_birth() = 0;

    /** Get thermal lifespan */
    virtual double get_lifespan() = 0;

    /** Get mixing layer thickness */
    virtual double get_zi() = 0;

    /** Get thermal model */
    virtual int get_model() = 0;

    /** Get shape parameter */
    virtual double get_ksi() = 0;

    /** Get vector of thermal centers */
    virtual std::vector<double> get_center() = 0;

    /**
     * @brief Return true if the thermal is alive, else false
     * @param {const double &} t; current time
     */
    virtual bool is_alive(const double &t) = 0;

    /**
     * @brief Return the thermal life cycle coefficient
     * @param {const double &} t; current time
     */
    virtual double lifetime_coefficient(const double &t) = 0;

    /**
     * @brief Get the Euclidean distance between a point (x,y,z) and the center of the thermal. If z =! 0 then it considers the drift of the thermal center at the height z.
     * @param {double} x, y, z; coordinate in the earth frame
	 */
    virtual double dist_to_updraft_center(const double &x, const double &y, const double &z) = 0;

    /**
     * @brief Set the wind speed onto the x-axis and y-axis.
     * @param {double} wx, wy; wind velocity vector coordinates in the earth frame
	 */
    void set_horizontal_wind(const double &wx, const double &wy) {windx=wx; windy=wy;}

	/**
     * @brief Computes the wind vector w, at point (x,y,z), at time t.
     * @param {double} x, y, z; coordinate in the earth frame
     * @param {double} t; time
     * @param {std::vector<double> &} w; wind velocity vector in the earth frame
	 */
	virtual thermal& wind(const double &x, const double &y, const double &z, const double &t, std::vector<double> &w) = 0;

	/** @brief Print the thermal's features in the standard output stream */
	virtual void print_std_os() = 0;
};

}

#endif