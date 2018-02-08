#ifndef L2FSIM_GP_MODEL_HPP_
#define L2FSIM_GP_MODEL_HPP_

#include <gaussian_process.hpp>

/**
 * @file gp_model.hpp
 * @version 1.0
 * @brief GP model for the flight zone based on 'flat_thermal_soaring_zone.hpp' flight zones
 */

namespace L2Fsim {

class gp_model : public flat_zone {
    /**
     * @brief Attributes
     * @param {flat_thermal_soaring_zone *} fz; pointer to a flight zone form which data points are extracted
     */
    flat_thermal_soaring_zone *fz;
    gaussian_process gp;

    /**
     * @brief Initialize (build) the model based on the given flight zone
     * @note The
     */
    void initialize() {
        for(auto &th : fz->thermals) {
            if(th->is_alive(0.)) {
                //TODO: add other criteria to consider less thermals eg vision constraints
                //TODO: get the position of the "top" rather than the center
                gp.add_point(th->get_center(), th->get_w_star());
            }
        }
    }

public :
    /** @brief Constructor */
    gp_model(
        flat_thermal_soaring_zone *_fz,
		double (*_kernel_function) (std::vector<double>, std::vector<double>),
		double _noise_var = 0.) :
		fz(_fz),
		gp(_kernel_function,_noise_var)
    {
        initialize();
    }

	/**
	 * @brief Compute the wind velocity vector w at coordinate (x, y, z, t)
     * @param {double} x, y, z, t; coordinates
     * @param {std::vector<double> &} w; wind velocity vector (wx, wy, wz)
     */
    gp_model& wind(double x, double y, double z, double t, std::vector<double> &w) {
        std::vector<double> pos = {x, y, z};
        double updraft = gp.predict_mean(pos);
        w = {0., 0., updraft}; // TODO add hozirontal components
        return *this;
        (void) t; //TODO add time dependency
    }

    /**
     * @brief Write the updraft values in a file for visualization
     * @param {const double} dx, dy; mesh precision
     * @param {const std::vector<double> &} z, t; altitudes and times of the saved updraft field
     * @param {const std::string &} op; output path
     */
    void save_updraft_values(
        const double dx,
        const double dy,
        const std::vector<double> &z_vec,
        const std::vector<double> &t_vec,
        const std::string &op)
    {
        std::ofstream ofs;
        std::string sep = ";";
        ofs.open(op);
        ofs<<"t"<<sep;
        ofs<<"x"<<sep;
        ofs<<"y"<<sep;
        ofs<<"z"<<sep;
        ofs<<"updraft"<<std::endl;
        std::vector<double> w(3);
        for(auto &z : z_vec) {
            for(auto &t : t_vec) {
                for(double x=-1500.; x<=1500.; x+=dx) {
                    for(double y=-1500.; y<=1500.; y+=dy) {
                        wind(x,y,z,t,w);
                        ofs << t << sep;
                        ofs << x << sep;
                        ofs << y << sep;
                        ofs << z << sep;
                        ofs << w.at(2);
                        ofs << std::endl;
                    }
                }
            }
        }
        ofs.close();
    }
};

}

#endif
