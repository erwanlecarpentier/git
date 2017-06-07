#ifndef L2FSIM_CFG_READER_HPP_
#define L2FSIM_CFG_READER_HPP_

#include <libconfig.h++>
#include <memory>

#include <L2Fsim/aircraft/aircraft.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_state.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_command.hpp>

#include <L2Fsim/flight_zone/flight_zone.hpp>
#include <L2Fsim/flight_zone/flat_thermal_soaring_zone.hpp>
#include <L2Fsim/flight_zone/flat_zone.hpp>

#include <L2Fsim/stepper/stepper.hpp>
#include <L2Fsim/stepper/euler_integrator.hpp>
#include <L2Fsim/stepper/rk4_integrator.hpp>

#include <L2Fsim/pilot/pilot.hpp>
#include <L2Fsim/pilot/passive_pilot.hpp>
#include <L2Fsim/pilot/heuristic_pilot.hpp>
#include <L2Fsim/pilot/q_learning/q_learning_pilot.hpp>
#include <L2Fsim/pilot/uct/b03_uct_pilot.hpp>
//#include <L2Fsim/pilot/optimistic/optimistic_pilot.hpp> // TODO uncomment

/**
 * @file cfg_reader.hpp
 * @brief Configuration reader
 * @version 1.0
 */

namespace L2Fsim {

struct cfg_reader {
    /** @brief Constructor */
    cfg_reader() {}

    void disp_err() {
        std::cout << "Error: variable initialisation in config file" << std::endl;
    }

    /** @brief Read the path to the state log file */
    std::string read_st_log_path(const libconfig::Config &cfg) {
        if(cfg.exists("st_log_path")) {return cfg.lookup("st_log_path");}
        else {disp_err();}
        return nullptr;
    }

    /** @brief Read the path to the envt log file */
    std::string read_fz_log_path(const libconfig::Config &cfg) {
        if(cfg.exists("fz_log_path")) {return cfg.lookup("fz_log_path");}
        else {disp_err();}
        return nullptr;
    }

    /** @brief Read and initialise the time variables */
    void read_time_variables(const libconfig::Config &cfg, double &t_lim, double &Dt, double &nb_dt) {
        if(cfg.lookupValue("limit_time",t_lim)
        && cfg.lookupValue("time_step_width",Dt)
        && cfg.lookupValue("nb_sub_time_step",nb_dt)) {/* nothing to do */}
        else {disp_err();}
    }

    /** @brief Read and initialise an environment */
    std::unique_ptr<flight_zone> read_environment(const libconfig::Config &cfg) {
        unsigned int sl=0;
        if(cfg.lookupValue("envt_selector", sl)) {
            switch(sl) {
            case 0: { // flat_zone
                double wx=0., wy=0.;
                if(cfg.lookupValue("wx", wx) && cfg.lookupValue("wy", wy)) {
                    return std::unique_ptr<flight_zone> (new flat_zone(wx,wy));
                } else {disp_err();}
            }
            case 1: { // flat_thermal_soaring_zone
                std::string sc_path = "config/thermal_scenario.csv";
                std::string envt_cfg_path = "config/fz_cfg.csv";
                double noise_stddev=0.;
                if (cfg.lookupValue("th_scenario_path", sc_path) &&
                    cfg.lookupValue("envt_cfg_path", envt_cfg_path) &&
                    cfg.lookupValue("noise_stddev", noise_stddev)) {
                    return std::unique_ptr<flight_zone> (new flat_thermal_soaring_zone(sc_path,envt_cfg_path,noise_stddev));
                } else {disp_err();}
            }
            }
        }
        else {disp_err();}
        return std::unique_ptr<flight_zone> (nullptr);
    }

    /** @brief Read and set the values of the state variables */
    void read_state(
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
        && cfg.exists("gamma0")
        && cfg.exists("khi0")
        && cfg.exists("alpha0")
        && cfg.exists("beta0")
        && cfg.exists("sigma0")
        && cfg.exists("maximum_angle_magnitude")) {
            _gamma0 = cfg.lookup("gamma0"); _gamma0 *= TO_RAD;
            _khi0 = cfg.lookup("khi0"); _khi0 *= TO_RAD;
            _alpha0 = cfg.lookup("alpha0"); _alpha0 *= TO_RAD;
            _beta0 = cfg.lookup("beta0"); _beta0 *= TO_RAD;
            _sigma0 = cfg.lookup("sigma0"); _sigma0 *= TO_RAD;
            _mam = cfg.lookup("maximum_angle_magnitude"); _mam *= TO_RAD;
        }
        else {disp_err();}
    }

    std::unique_ptr<aircraft> read_aircraft(const libconfig::Config &cfg) {
        if(cfg.exists("aircraft_selector")) {
            unsigned int sl = cfg.lookup("aircraft_selector");
            switch(sl) {
            case 0: { // beeler_glider
                double x0=0., y0=0., z0=0., V0=0., gamma0=0., khi0=0., alpha0=0., beta0=0., sigma0=0., mam=0.;
                read_state(cfg,x0,y0,z0,V0,gamma0,khi0,alpha0,beta0,sigma0,mam);
                beeler_glider_state s(x0,y0,z0,V0,gamma0,khi0,alpha0,beta0,sigma0,mam);
                beeler_glider_command a;
                return std::unique_ptr<aircraft> (new beeler_glider(s,a));
            }
            default: {disp_err();}
            }
        }
        else {disp_err();}
        return std::unique_ptr<aircraft> (nullptr);
    }

    /** @brief Read and initialise a stepper */
    std::unique_ptr<stepper> read_stepper(const libconfig::Config &cfg, const double &sub_dt) {
        if(cfg.exists("stepper_selector")) {
            unsigned int sl = cfg.lookup("stepper_selector");
            switch(sl) {
            case 0: { // euler_integrator
                return std::unique_ptr<stepper> (new euler_integrator(sub_dt));
            }
            case 1: { // rk4_integrator
                return std::unique_ptr<stepper> (new rk4_integrator(sub_dt));
            }
            }
        }
        else {disp_err();}
        return std::unique_ptr<stepper> (nullptr);
    }

    /** @brief Read and initialise a pilot
     * @todo write a method for each case
     */
    std::unique_ptr<pilot> read_pilot(const libconfig::Config &cfg) {
        if(cfg.exists("pilot_selector")) {
            unsigned int sl = cfg.lookup("pilot_selector");
            switch(sl) {
            case 0: { // passive_pilot
                double arm=.1;
                if(cfg.lookupValue("angle_rate_magnitude",arm)) {
                    return std::unique_ptr<pilot> (new passive_pilot(arm));
                } else {disp_err();}
            }
            case 1: { // heuristic_pilot
                double arm=.1;
                if(cfg.lookupValue("angle_rate_magnitude",arm)) {
                    return std::unique_ptr<pilot> (new heuristic_pilot(arm));
                } else {disp_err();}
            }
            case 2: { // q_learning_pilot
                double arm=.1, ep=.1, lr=.01, df=.9;
                if (cfg.lookupValue("angle_rate_magnitude",arm),
                    cfg.lookupValue("q_epsilon",ep),
                    cfg.lookupValue("q_learning_rate",lr),
                    cfg.lookupValue("q_discount_factor",df))
                {
                    return std::unique_ptr<pilot> (new q_learning_pilot(arm,ep,lr,df));
                } else {disp_err();}
            }
            case 3: { // b03_uct_pilot
                std::string sc_path, envt_cfg_path;
                double noise_stddev=0., arm=.1, pr=.7, dt=.1, sdt=.1, df=.9;
                unsigned int hz=100, bd=1000;
                if(cfg.lookupValue("th_scenario_path", sc_path)
                && cfg.lookupValue("envt_cfg_path", envt_cfg_path)
                && cfg.lookupValue("noise_stddev", noise_stddev)
                && cfg.lookupValue("angle_rate_magnitude",arm)
                && cfg.lookupValue("uct_parameter",pr)
                && cfg.lookupValue("uct_time_step_width",dt)
                && cfg.lookupValue("uct_sub_time_step_width",sdt)
                && cfg.lookupValue("uct_discount_factor",df)
                && cfg.lookupValue("uct_horizon",hz)
                && cfg.lookupValue("uct_budget",bd))
                {
                    // 1. aircraft model
                    double x0=0., y0=0., z0=0., V0=0., gamma0=0., khi0=0., alpha0=0., beta0=0., sigma0=0., mam=0.;
                    read_state(cfg,x0,y0,z0,V0,gamma0,khi0,alpha0,beta0,sigma0,mam);
                    beeler_glider_state s(x0,y0,z0,V0,gamma0,khi0,alpha0,beta0,sigma0,mam);
                    beeler_glider_command a;
                    beeler_glider ac_model(s,a);

                    // 2. flight zone model
                    flat_thermal_soaring_zone fz_model(sc_path,envt_cfg_path,noise_stddev);

                    // 3. transition function
                    //euler_integrator st(sdt);//TRM

                    return std::unique_ptr<pilot> (new b03_uct_pilot(
                        ac_model, fz_model,
                        //st.transition_function,
                        euler_integrator::transition_function,
                        arm, pr, dt, sdt, df, hz, bd));
                } else {disp_err();}
            }
            case 4: { // optimistic_pilot
                /* TODO
                double time_step_width=0.;
                if(cfg.lookupValue("opt_time_step_width",time_step_width)) {
                    return std::unique_ptr<pilot> (new optimistic_pilot(time_step_width));
                } else {disp_err();}
                */
            }
            }
        }
        else {disp_err();}
        return std::unique_ptr<pilot> (nullptr);
    }
};

}

#endif
