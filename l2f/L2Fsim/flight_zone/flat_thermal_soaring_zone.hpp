#ifndef L2FSIM_FLAT_THERMAL_SOARING_ZONE_HPP_
#define L2FSIM_FLAT_THERMAL_SOARING_ZONE_HPP_

#include <L2Fsim/flight_zone/flight_zone.hpp>
#include <L2Fsim/flight_zone/flat_zone.hpp>
#include <L2Fsim/flight_zone/thermal/std_thermal.hpp>
#include <string>
#include <fstream>
#include <sstream>

namespace L2Fsim {

/**
 * @file flat_thermal_soaring_zone.hpp
 * @version 1.1
 * @since 1.0
 *
 * @brief This class implements the components of the wind vector by introducing the thermals in the flat zone
 * @note The abstract class flat_thermal_soaring_zone is a subclass of flat_zone
 */

class flat_thermal_soaring_zone : public flat_zone
{
protected:
    /**
     * Attributes
     * @param {double} tstart; starting time of the simulation
     * @param {double} tend; ending time of the simulation
     * @param {int} minX, maxX, minY, maxY, minZ, maxZ; boundaries
     * @param {double} windx, windy; lateral components of the wind velocity
     * @param {double} dmin; radius of a thermal
     * @param {double} zi; height of a thermal
     * @param {std::vector<thermal *>} thermals; list of the thermals created in the simulation
     */
    double tstart=0.;
    double tend;
    int minX, maxX, minY, maxY, minZ, maxZ;
    double windx, windy;
    double dmin;
    double zi;
    std::vector<thermal *> thermals;

    int nbMaxThermals();
    bool createThermalCenter(std::vector<double>& center, double t);
    int numberAliveAtTime(double t);
    double environSink(double z, double t);

public:
    /**
     * Constructor
     * @brief create an empty zone defined by its dimension, wind, ending time and thermal's height
     */
    flat_thermal_soaring_zone(
    double _tend,
    int _minX,
    int _maxX,
    int _minY,
    int _maxY,
    int _minZ,
    int _maxZ,
    double _windx,
    double _windy,
    double _dmin,
    double _zi);

    /**
     * Constructor
     * @brief Read a saved thermal scenario in order to play an identical simulation of the flight zone
	 * @param {std::string} filename; path to the saved scenario
	 */
    flat_thermal_soaring_zone(std::string filename);

	/**
	 * @brief Compute the wind velocity vector w at coordinate (x,y,z,t)
	 * @param {double} x, y, z, t; coordinates
	 * @param {std::vector<double> &} w; wind velocity vector [wx, wy, wz]
	 */
    flat_thermal_soaring_zone& wind(double x, double y, double z, double t, std::vector<double> &w);

    /**
	 * @brief Create a scenario including thermals
	 * @param {double} deltaT, period of thermal actualization
	 * @param {int} model; thermal model
	 */
    void createScenario(double deltaT, int model);

    /**
	 * @brief Write the wind data for the visualization of a 'zslice' in a file
	 * @param {double} deltaT, period of thermal actualization
	 * @param {double} deltax, deltay; mesh precision
	 * @param {double} zslice; height of the windfield
	 * @param {std::string} filename; output path
	 */
    void writeScenario(double deltaT, double deltax, double deltay, double zslice, std::string filename);

    /**
	 * @brief Save a scenario in '.txt' format
	 * @param {std::string} filename; output path
	 */
    void saveConfig(std::string filename);

    /**
	 * @brief Save a scenario in '.csv' format
	 * @param {std::string} filename; output path
	 */
    void saveConfigToCSV(std::string filename);
};

}

#endif
