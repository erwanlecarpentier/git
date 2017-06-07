#ifndef L2FSIM_CFG_READER_HPP_
#define L2FSIM_CFG_READER_HPP_

#include <L2Fsim/flight_zone/flat_thermal_soaring_zone.hpp>
#include <L2Fsim/flight_zone/flat_zone.hpp>

#include <L2Fsim/stepper/euler_integrator.hpp>
#include <L2Fsim/stepper/rk4_integrator.hpp>

#include <L2Fsim/pilot/passive_pilot.hpp>
#include <L2Fsim/pilot/heuristic_pilot.hpp>
#include <L2Fsim/pilot/q_learning/q_learning_pilot.hpp>

#include <L2Fsim/flight_zone/flight_zone.hpp>
#include <L2Fsim/aircraft/aircraft.hpp>
#include <L2Fsim/stepper/stepper.hpp>
#include <L2Fsim/pilot/pilot.hpp>

#include <libconfig.h++>

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
        unsigned int sl=0;
        if(cfg.lookupValue("envt_selector", sl)) {
            switch(sl) {
            case 0: { // flat_zone
                double wx=0., wy=0.;
                if(cfg.lookupValue("wx", wx) && cfg.lookupValue("wy", wy)) {
                    return new flat_zone(wx,wy);
                }else {display_error_msg();}
            }
            case 1: { // flat_thermal_soaring_zone
                std::string sc_path, envt_cfg_path;
                double noise_stddev=0.;
                if (cfg.lookupValue("th_scenario_path", sc_path) &&
                    cfg.lookupValue("envt_cfg_path", envt_cfg_path) &&
                    cfg.lookupValue("noise_stddev", noise_stddev)) {
                    return new flat_thermal_soaring_zone(sc_path,envt_cfg_path,noise_stddev);
                }else {display_error_msg();}
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
        double &_sigma0,
        double &_mam)
    {
        if(cfg.lookupValue("x0", _x0)
        && cfg.lookupValue("y0", _y0)
        && cfg.lookupValue("z0", _z0)
        && cfg.lookupValue("V0", _V0)
        && cfg.lookupValue("gamma0", _gamma0)
        && cfg.lookupValue("khi0", _khi0)
        && cfg.lookupValue("alpha0", _alpha0)
        && cfg.lookupValue("beta0", _beta0)
        && cfg.lookupValue("sigma0", _sigma0)
        && cfg.lookupValue("maximum_angle_magnitude", _mam)) {
            _x0 = cfg.lookup("x0");
            _y0 = cfg.lookup("y0");
            _z0 = cfg.lookup("z0");
            _V0 = cfg.lookup("V0");
            _gamma0 = cfg.lookup("gamma0"); _gamma0 *= TO_RAD;
            _khi0 = cfg.lookup("khi0"); _khi0 *= TO_RAD;
            _alpha0 = cfg.lookup("alpha0"); _alpha0 *= TO_RAD;
            _beta0 = cfg.lookup("beta0"); _beta0 *= TO_RAD;
            _sigma0 = cfg.lookup("sigma0"); _sigma0 *= TO_RAD;
            _mam = cfg.lookup("maximum_angle_magnitude"); _mam *= TO_RAD;
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

    /** @brief Read and initialise a pilot
     * @todo write a method for each case
     */
    pilot * read_pilot_variable(const libconfig::Config &cfg) {
        if(cfg.exists("pilot_selector")) {
            unsigned int sl = cfg.lookup("pilot_selector");
            switch(sl) {
            case 0: { // passive_pilot
                double arm=.1;
                if(cfg.lookupValue("angle_rate_magnitude",arm)) {
                    return new passive_pilot(arm);
                } else {display_error_msg();}
            }
            case 1: { // heuristic_pilot
                double arm=.1;
                if(cfg.lookupValue("angle_rate_magnitude",arm)) {
                    return new heuristic_pilot(arm);
                } else {display_error_msg();}
            }
            case 2: { // q_learning_pilot
                double arm=.1, ep=.1, lr=.01, df=.9;
                if (cfg.lookupValue("angle_rate_magnitude",arm),
                    cfg.lookupValue("q_epsilon",ep),
                    cfg.lookupValue("q_learning_rate",lr),
                    cfg.lookupValue("q_discount_factor",df))
                {
                    return new q_learning_pilot(arm,ep,lr,df);
                } else {display_error_msg();}
            }
            case 3: { // b03_uct_pilot
                /* TODO

                // envt
                unsigned int envt_sl=0;
                if(cfg.lookupValue("uct_envt_selector",envt_sl)) {
                    if(envt_sl == 0) {
                        //todo
                    }
                } else {display_error_msg();}







                double arm=.1, pr=.7, dt=.1, sdt=.1, df=.9, hz=100., bd=1e4;
                if (cfg.lookupValue("angle_rate_magnitude",arm),
                    cfg.lookupValue("uct_parameter",pr),
                    cfg.lookupValue("uct_time_step_width",dt),
                    cfg.lookupValue("uct_sub_time_step_width",sdt),
                    cfg.lookupValue("uct_discount_factor",df),
                    cfg.lookupValue("uct_horizon",hz),
                    cfg.lookupValue("uct_budget",bd))
                {
                    return new b03_uct_pilot(
                        my_glider, fz,
                        uct_stepper.transition_function,
                        arm, pr, dt, sdt, df, hz, bd);
                } else {display_error_msg();}
                */
            }
            }
        }
        else {display_error_msg();}
        return NULL;
    }
};

}

#endif
