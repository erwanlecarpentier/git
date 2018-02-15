#ifndef L2FSIM_FLAT_THERMAL_SOARING_ZONE_HPP_
#define L2FSIM_FLAT_THERMAL_SOARING_ZONE_HPP_

#include <thermal/std_thermal.hpp>
#include <chrono>

/**
 * @file flat_thermal_soaring_zone.hpp
 * @version 1.1
 * @since 1.0
 * @brief Flat thermal soaring zone
 *
 * This class implements the components of the wind vector by introducing the thermals in the flat zone
 * The abstract class flat_thermal_soaring_zone is a subclass of flat_zone.
 * Default setting is noiseless.
 * No seed selection is done - you would have to implement this if you want to re-generate matching pseudo-random sequences
 */

namespace L2Fsim {

class flat_thermal_soaring_zone : public flat_zone {
public:
    double t_start = 0.; ///< Starting time of the simulation
    double t_limit; ///< Ending time of the simulation
    double windx, windy; ///< Horizontal components of the wind velocity
    double w_star_min, w_star_max; ///< Minimum and maximum average updraft velocity
    double zi_min, zi_max; ///< Minimum and maximum mixing layer thickness
    double lifespan_min, lifespan_max; ///< Minimum and maximum lifespan
    double x_min, x_max, y_min, y_max, z_min, z_max; ///< Boundaries
    double ksi_min, ksi_max; ///< Minimum and maximum roll-off parameters
    double d_min; ///< Minimum radius of a thermal
    unsigned int nbth; ///< Maximum number of thermals in the scenario
    std::vector<thermal *> thermals; ///< List of the thermals created in the simulation
    double noise_stddev = 0.; ///< Standard deviation of the normal law whose samples are added to each component of the wind velocity vector

    /**
     * @brief Constructor
     *
     * Create an empty zone defined by its extremum dimensions.
     */
    flat_thermal_soaring_zone(
        double _t_start,
        double _t_limit,
        double _windx,
        double _windy,
        double _w_star_min,
        double _w_star_max,
        double _zi_min,
        double _zi_max,
        double _lifespan_min,
        double _lifespan_max,
        double _x_min,
        double _x_max,
        double _y_min,
        double _y_max,
        double _z_min,
        double _z_max,
        double _ksi_min,
        double _ksi_max,
        double _d_min,
        unsigned int _nbth) :
        t_start(_t_start),
        t_limit(_t_limit),
        windx(_windx),
        windy(_windy),
        w_star_min(_w_star_min),
        w_star_max(_w_star_max),
        zi_min(_zi_min),
        zi_max(_zi_max),
        lifespan_min(_lifespan_min),
        lifespan_max(_lifespan_max),
        x_min(_x_min),
        x_max(_x_max),
        y_min(_y_min),
        y_max(_y_max),
        z_min(_z_min),
        z_max(_z_max),
        ksi_min(_ksi_min),
        ksi_max(_ksi_max),
        d_min(_d_min),
        nbth(_nbth)
    {}

    /**
     * @brief Constructor
     *
     * Read a pre-saved thermal scenario (method save_scenario) in order to play an identical simulation.
     * @warning Expect same order of variables as in save_scenario and save_fz_cfg method, do not modify one without the other.
     * @param {std::string} th_sc_p; thermal scenario input path
     * @param {std::string} fz_cfg_p; flight zone configuration input path
     */
    flat_thermal_soaring_zone(std::string th_sc_p, std::string fz_cfg_p, double _noise_stddev=0.) :
        noise_stddev(_noise_stddev)
    {
        // 1. Read flight zone configuration
        std::ifstream cf_file(fz_cfg_p);
        if (!cf_file.is_open()) {
            std::cerr << "Unable to open input file ("<< fz_cfg_p <<") in flat_thermal_soaring_zone constructor" << std::endl;
        }
        std::string line;
        std::getline(cf_file,line); // do not take first line into account
        std::getline(cf_file,line); // take 2nd line
        if(line.size()>0) { // prevent from reading empty line
            std::vector<std::string> result;
            std::stringstream line_stream(line);
            std::string cell;
            while(std::getline(line_stream,cell,';')) {
                result.push_back(cell);
            }
            if (!line_stream && cell.empty()) { // This checks for a trailing comma with no data after it
                // If there was a trailing comma then add an empty element
                result.push_back("");
            }
            x_min = stod(result.at(0));
            x_max = stod(result.at(1));
            y_min = stod(result.at(2));
            y_max = stod(result.at(3));
            z_min = stod(result.at(4));
            z_max = stod(result.at(5));
            d_min = stod(result.at(6));
            windx = stod(result.at(7));
            windy = stod(result.at(8));
        } else {std::cout << "Unable to read 2nd line in fz configuration reader in flat_thermal_soaring_zone constructor"<<std::endl;}
        cf_file.close();
        // 2. Read thermal scenario
        std::ifstream sc_file(th_sc_p);
        if (!sc_file.is_open()) {
            std::cerr << "Unable to open input file ("<< th_sc_p <<") in flat_thermal_soaring_zone constructor" << std::endl;
        }
        std::getline(sc_file,line); // do not take first line into account
        while(sc_file.good()) {
            std::vector<std::string> result;
            std::getline(sc_file,line);
            if(line.size()>0) { // prevent from reading last (empty) line
                std::stringstream line_stream(line);
                std::string cell;
                while(std::getline(line_stream,cell,';')) {
                    result.push_back(cell);
                }
                if (!line_stream && cell.empty()) { // This checks for a trailing comma with no data after it
                    // If there was a trailing comma then add an empty element
                    result.push_back("");
                }
                int model = std::stoi(result.at(0));
                double t_birth = std::stod(result.at(1));
                double lifespan = std::stod(result.at(2));
                double w_star = std::stod(result.at(3));
                double zi = std::stod(result.at(4));
                double x = std::stod(result.at(5));
                double y = std::stod(result.at(6));
                double z = std::stod(result.at(7));
                double ksi = std::stod(result.at(8));
                std_thermal *new_th = new std_thermal(model,w_star,zi,t_birth,lifespan,x,y,z,ksi);
                new_th->set_horizontal_wind(windx,windy);
                thermals.push_back(new_th);
            }
        }
        sc_file.close();
    }

    /** @brief Destructor */
    ~flat_thermal_soaring_zone() {
        for(auto th : thermals) {delete th;}
    }

    /**
     * @brief Create thermal center
     *
     * Define the center of a new thermal depending on positions of other thermals.
     * The new center is randomly picked within [x_min,x_max]*[y_min,y_max] and its validity
     * is ensured.
     * @param {double} t; time
     * @param {std::vector<double> &} center; computed center
     * @return Return true if a valid center is found.
     */
    bool create_thermal_center(double t, std::vector<double> &center) {
        double x_new=0., y_new=0.;
        unsigned int counter = 0;
        bool center_is_valid = false;
        while(!center_is_valid) {
            counter++;
            x_new = uniform_double(x_min,x_max);
            y_new = uniform_double(y_min,y_max);
            if(thermals.size()>0) { // compare picked center to other thermals
                std::vector<double> distances;
                for(auto& th : thermals) { // compute the distances to other alive thermals
                    if(th->is_alive(t)) {
                        double x_th = th->get_center()[0];
                        double y_th = th->get_center()[1];
                        double dist_to_th = sqrt((x_new-x_th)*(x_new-x_th)+(y_new-y_th)*(y_new-y_th));
                        distances.push_back(dist_to_th);
                    }
                }
                double minimum_distance = *std::min_element(distances.begin(),distances.end()); // #include <algorithm>
                if(minimum_distance>2.*d_min) {center_is_valid=true;}
            } else {center_is_valid=true;}
            if(counter>100) {
                std::cout<<"Warning: more than 100 trials were performed to create a new thermal center; ";
                std::cout<<"Make sure that you are asking something possible (d_min = "<<d_min<<")"<<std::endl;
            }
        }
        center.clear();
        center.push_back(x_new);
        center.push_back(y_new);
        center.push_back(0.);
        return center_is_valid;
    }

    /**
     * @brief Number of alive thermal
     *
     * Count the number of alive thermals at time t.
     * @param {double} t; time
     * @return Return the number of alive thermal at time t.
     */
    int nb_th_alive_at_time(double t) {
        int counter = 0;
        for(auto &th : thermals) {if(th->is_alive(t)) {++counter;}}
        return counter;
    }

    /**
     * @brief Sink rate
     *
     * Compute the global environment sink rate.
     * @param {double} z, t; altitude and time
     * @return Return the global sink rate.
     */
    double global_sink_rate(double z, double t) {
        double thermals_area=0., mass_flow=0.;
        for(auto &th : thermals) {
            if (th->is_alive(t)) {
                double z_zi = z / th->get_zi();
                double s_wd_th = ((.5 < (z_zi)) && ((z_zi) < .9)) ? 2.5*(z_zi - .5) : 0.;
                double avg_updraft_th = th->get_w_star() * pow(z_zi,1./3.) * (1. - 1.1*z_zi);
                double radius_th = .102 * pow((z_zi),1./3.) * (1 - .25*z_zi) * (z/z_zi);
                if(radius_th<10.){radius_th=10.;}
                double rsq = radius_th*radius_th;
                mass_flow += avg_updraft_th*M_PI*rsq*(1.-s_wd_th) * th->lifetime_coefficient(t);
                thermals_area += M_PI*rsq;
            }
        }
        double w_e = - mass_flow / ((x_max-x_min)*(y_max-y_min) - thermals_area);
        if(w_e>0.){w_e=0.;}
        return w_e;
    }

    /**
     * @brief Pick w_star
     *
     * Return w_star in range [w_star_min,w_star_max].
     * This is used for thermal creation.
     * @return Return w_star
     */
    double pick_w_star() {
        return uniform_double(w_star_min,w_star_max);
    }

    /**
     * @brief Pick zi
     *
     * Return zi in range [zi_min,zi_max].
     * This is used for thermal creation.
     * @return Return zi.
     */
    double pick_zi() {//(double w_star) {
        //double noise = uniform_double(-50.,50.);
        //return 961.0264191 * w_star - 701.9624694 + noise;
        return uniform_double(zi_min,zi_max);
    }

    /**
     * @brief Pick lifespan
     *
     * Return lifespan as a function of w_star.
     * This is used for thermal creation.
     * @param {double} w_star; given w_star
     * @return Return lifespan.
     */
    double pick_lifespan(double w_star) {
        double a = (lifespan_max - lifespan_min) / (w_star_max - w_star_min);
        double b = lifespan_min - a * w_star_min;
        return a * w_star + b;
    }

    /**
     * @brief Pick ksi
     *
     * Return ksi in range [ksi_min,ksi_max].
     * This is used for thermal creation.
     * @return Return ksi.
     */
    double pick_ksi() {
        return uniform_double(ksi_min,ksi_max);
    }

    /**
     * @brief Print scenario
     *
     * Print the full scenario in the standard output stream
     */
    void print_scenario() {
        for(auto &th : thermals) {th->print();}
    }

    /**
     * @brief Get total number of thermals
     *
     * Get the total number of thermals in the whole scenario.
     * @return Return the total number of thermals.
     */
    unsigned int get_total_nb_of_th() {
        return thermals.size();
    }

	/**
     * @brief Wind
     *
     * Compute the wind velocity vector w at coordinate (x,y,z,t).
     * @param {double} x, y, z, t; coordinates
     * @param {std::vector<double> &} w; wind velocity vector [wx, wy, wz]
     * @return Return '*this'
     */
    flat_thermal_soaring_zone& wind(double x, double y, double z, double t, std::vector<double> &w) override {
        w.assign({windx,windy,0.});
        for(auto &th : thermals) {
            if(th->is_alive(t)) {
                th->wind(x,y,z,t,w);
            }
        }
        if(thermals.size()!=0 && thermals[0]->get_model()==1){
            w[2] += global_sink_rate(z,t);
        }
        if (!are_equal(noise_stddev,0.)) {
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator (seed);
            std::normal_distribution<double> distribution(0.,noise_stddev);
            //w[0] += distribution(generator);
            //w[1] += distribution(generator);
            w[2] += distribution(generator);
        }
        return *this;
    }

    /**
     * @brief Is within flightzone
     *
     * Assert that the aircraft is inside the flight zone.
     * @param {double} x, y, z; coordinates  in the earth frame
     * @return Return true if the input position belongs to the flight zone.
     */
    virtual bool is_within_fz(double x, double y, double z) override {
        (void) z; //unused by default
        if (x_min<x && x<x_max && y_min<y && y<y_max) {return true;}
        return false;
    }

    /**
     * @brief Create thermal
     *
     * Create a new thermal and push it back to 'thermals' vector attribute.
     * @param {int} model; thermal model
     * @param {double} t; current time, thermal's date of birth
     */
    void create_thermal(int model, double t) {
        std::vector<double> center;
        if(create_thermal_center(t,center)) {
            double w_star = pick_w_star();
            double zi = pick_zi();
            double lifespan = pick_lifespan(w_star);
            double ksi = pick_ksi();
            std_thermal *new_th = new std_thermal(model,w_star,zi,t,lifespan,center.at(0),center.at(1),center.at(2),ksi);
            new_th->set_horizontal_wind(windx,windy);
            thermals.push_back(new_th);
        }
    }

    /**
     * @brief Create scenario
     *
     * Create a scenario i.e. fill the 'thermals' vector attribute.
     * @param {double} dt, refresh rate
     * @param {int} model; thermal model
     */
    void create_scenario(double dt, int model) {
        for(double t=t_start; t<=t_limit; t+=dt) {
            while(nb_th_alive_at_time(t) <= (double)nbth) {
                create_thermal(model,t);
            }
        }
    }

    /**
     * @brief Save updraft values
     *
     * Write the updraft values in a file for visualization
     * @param {double} dx, dy; mesh precision
     * @param {const std::vector<double> &} z, t; altitudes and times of the saved updraft
     * field
     * @param {const std::string &} op; output path
     */
    void save_updraft_values(
        double dx,
        double dy,
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
                for(double x=x_min; x<=x_max; x+=dx) {
                    for(double y=y_min; y<=y_max; y+=dy) {
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

    /**
     * @brief Save scenario
     *
     * Save a scenario at the specified output path.
     * @param {std::string} op; output path
     */
    void save_scenario(std::string op) {
        std::ofstream ofs;
        std::string sep = ";";
        ofs.open(op);
        ofs<<"model"<<sep;
        ofs<<"t_birth"<<sep;
        ofs<<"lifespan"<<sep;
        ofs<<"w_star"<<sep;
        ofs<<"zi"<<sep;
        ofs<<"x"<<sep;
        ofs<<"y"<<sep;
        ofs<<"z"<<sep;
        ofs<<"ksi"<<std::endl;
        for(auto &th : thermals) {
            std::vector<double> center = th->get_center();
            ofs<<th->get_model()<<sep;
            ofs<<th->get_t_birth()<<sep;
            ofs<<th->get_lifespan()<<sep;
            ofs<<th->get_w_star()<<sep;
            ofs<<th->get_zi()<<sep;
            ofs<<center.at(0)<<sep;
            ofs<<center.at(1)<<sep;
            ofs<<center.at(2)<<sep;
            ofs<<th->get_ksi();
            ofs<<std::endl;
        }
    }

    /**
     * @brief Save flight zone configuration
     *
     * Save the flight zone dimensions, d_min and horizontal wind velocities.
     * @param {std::string} op; output path
     */
    void save_fz_cfg(std::string op) {
        std::ofstream ofs;
        std::string sep = ";";
        ofs.open(op);
        ofs<<"x_min"<<sep;
        ofs<<"x_max"<<sep;
        ofs<<"y_min"<<sep;
        ofs<<"y_max"<<sep;
        ofs<<"z_min"<<sep;
        ofs<<"z_max"<<sep;
        ofs<<"d_min"<<sep;
        ofs<<"windx"<<sep;
        ofs<<"windy"<<std::endl;
        ofs<<x_min<<sep;
        ofs<<x_max<<sep;
        ofs<<y_min<<sep;
        ofs<<y_max<<sep;
        ofs<<z_min<<sep;
        ofs<<z_max<<sep;
        ofs<<d_min<<sep;
        ofs<<windx<<sep;
        ofs<<windy<<std::endl;
    }
};

}

#endif // end L2FSIM_FLAT_THERMAL_SOARING_ZONE_HPP_
