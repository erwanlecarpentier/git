#ifndef L2FSIM_CFG_READER_HPP_
#define L2FSIM_CFG_READER_HPP_

#include <L2Fsim/flight_zone/flat_thermal_soaring_zone.hpp>
#include <L2Fsim/flight_zone/flat_zone.hpp>

#include <L2Fsim/stepper/euler_integrator.hpp>
#include <L2Fsim/stepper/rk4_integrator.hpp>

#include <L2Fsim/flight_zone/flight_zone.hpp>
#include <L2Fsim/aircraft/aircraft.hpp>
#include <L2Fsim/stepper/stepper.hpp>
#include <L2Fsim/pilot/pilot.hpp>
#include <libconfig.h++>

#define TO_RAD 0.01745329251

/**
 * @file cfg_reader.hpp
 * @brief Configuration reader
 * @version 1.0
 */

namespace L2Fsim {

struct cfg_reader {
    /** @brief Constructor */
    cfg_reader() {}

    void display_error_msg() {
        std::cout << "Error: variable initialisation in config file" << std::endl;
    }

    /** @brief Read and initialise the time variables */
    void read_time_variables(const libconfig::Config &cfg, double &t_lim, double &Dt, double &nb_dt) {
        if(cfg.exists("limit_time")
        && cfg.exists("time_step_width")
        && cfg.exists("nb_sub_time_step")) {
            t_lim = cfg.lookup("limit_time");
            Dt = cfg.lookup("time_step_width");
            nb_dt = cfg.lookup("nb_sub_time_step");
        }
        else {display_error_msg();}
    }

    /** @brief Read and initialise an environment */
    flight_zone * read_environment_variables(const libconfig::Config &cfg) {
        if(cfg.exists("envt_selector")
        && cfg.exists("wx")
        && cfg.exists("wy")
        && cfg.exists("envt_path")) {
            unsigned int sl = cfg.lookup("envt_selector");
            switch(sl) {
            case 0: { // flat_zone
                double wx = cfg.lookup("wx");
                double wy = cfg.lookup("wy");
                return new flat_zone(wx,wy);
            }
            case 1: { // flat_thermal_soaring_zone
                std::string path = cfg.lookup("envt_path");
                return new flat_thermal_soaring_zone(path);
            }
            }
        }
        else {display_error_msg();}
        return NULL;
    }

    /** @brief Read and initialise the state variables */
    void read_state_variables(
        const libconfig::Config &cfg,
        double &_x0,
        double &_y0,
        double &_z0,
        double &_V0,
        double &_gamma0,
        double &_khi0,
        double &_alpha0,
        double &_beta0,
        double &_sigma0)
    {
        if(cfg.lookupValue("x0", _x0)
        && cfg.lookupValue("y0", _y0)
        && cfg.lookupValue("z0", _z0)
        && cfg.lookupValue("V0", _V0)
        && cfg.lookupValue("gamma0", _gamma0)
        && cfg.lookupValue("khi0", _khi0)
        && cfg.lookupValue("alpha0", _alpha0)
        && cfg.lookupValue("beta0", _beta0)
        && cfg.lookupValue("sigma0", _sigma0)) {
            _x0 = cfg.lookup("x0");
            _y0 = cfg.lookup("y0");
            _z0 = cfg.lookup("z0");
            _V0 = cfg.lookup("V0");
            _gamma0 = cfg.lookup("gamma0"); _gamma0 *= TO_RAD;
            _khi0 = cfg.lookup("khi0"); _khi0 *= TO_RAD;
            _alpha0 = cfg.lookup("alpha0"); _alpha0 *= TO_RAD;
            _beta0 = cfg.lookup("beta0"); _beta0 *= TO_RAD;
            _sigma0 = cfg.lookup("sigma0"); _sigma0 *= TO_RAD;
        }
        else {display_error_msg();}
    }

    /** @brief Read and initialise a stepper */
    stepper * read_stepper_variable(const libconfig::Config &cfg, const double &sub_dt) {
        if(cfg.exists("stepper_selector")) {
            unsigned int sl = cfg.lookup("stepper_selector");
            switch(sl) {
            case 0: { // euler_integrator
                return new euler_integrator(sub_dt);
            }
            case 1: { // rk4_integrator
                return new rk4_integrator(sub_dt);
            }
            }
        }
        else {display_error_msg();}
        return NULL;
    }
};

}

#endif
