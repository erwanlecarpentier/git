#ifndef L2FSIM_FLAT_THERMAL_SOARING_ZONE_HPP_
#define L2FSIM_FLAT_THERMAL_SOARING_ZONE_HPP_

#include <L2Fsim/flight_zone/flight_zone.hpp>
#include <L2Fsim/flight_zone/flat_zone.hpp>
#include <L2Fsim/flight_zone/thermal/std_thermal.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <chrono>

/**
 * @file flat_thermal_soaring_zone.hpp
 * @version 1.1
 * @since 1.0
 *
 * @brief This class implements the components of the wind vector by introducing the thermals in the flat zone
 * @note The abstract class flat_thermal_soaring_zone is a subclass of flat_zone
 */

namespace L2Fsim {

class flat_thermal_soaring_zone : public flat_zone {
public:
    /**
     * @brief Attributes
     * @param {double} t_start; starting time of the simulation
     * @param {double} t_limit; ending time of the simulation
     * @param {double} windx, windy; horizontal components of the wind velocity
     * @param {double} w_star_min, w_star_max; minimum and maximum average updraft velocity
     * @param {double} zi_min, zi_max; minimum and maximum mixing layer thickness
     * @param {double} lifespan_min, lifespan_max; minimum and maximum lifespan
     * @param {double} x_min, x_max, y_min, y_max, z_min, z_max; boundaries
     * @param {double} ksi_min, ksi_max; minimum and maximum roll-off parameter
     * @param {double} d_min; minimum radius of a thermal
     * @param {unsigned int} nbth; maximum number of thermals in the scenario
     * @param {std::vector<thermal *>} thermals; list of the thermals created in the simulation
     * @param {double} noise_stddev; standard deviation of the normal law whose samples are added to each component of the wind velocity vector
     * @note No seed selection is done - you would have to implement this if you want to re-generate matching pseudo-random sequences
     * @note Default setting is noiseless
     */
    double t_start = 0.;
    double t_limit;
    double windx, windy;
    double w_star_min, w_star_max;
    double zi_min, zi_max;
    double lifespan_min, lifespan_max;
    double x_min, x_max, y_min, y_max, z_min, z_max;
    double ksi_min, ksi_max;
    double d_min;
    unsigned int nbth;
    std::vector<thermal *> thermals;
    double noise_stddev = 0.;

    /** @brief Constructor; create an empty zone defined by its extremum dimensions */
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
     * @brief Constructor; Read a pre-saved thermal scenario (method save_scenario) in order to play an identical simulation
     * @warning Expect same order of variables as in save_scenario and save_fz_cfg method; do not modify one without the other
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

    /**
     * @brief Return the maximum number of thermals in the zone
     * @deprecated This method is not used anymore, we rather use nbth that defines this number directly
     */
    /*int get_max_nb_of_th() {
        double zi_avg = zi_max - zi_min;
        return floor(0.6*(x_max-x_min)*(y_max-y_min)/(d_min*zi_avg));
    }*/

    /**
     * @brief Define the center of a new thermal depending on positions of other thermals
     * @note The new center is randomly picked within [x_min,x_max]*[y_min,y_max] and its validity is ensured
     * @param {double} t; time
     * @param {std::vector<double> &} center; computed center
     */
    bool create_thermal_center(double t, std::vector<double> &center) {
        double x_new=0., y_new=0.;
        unsigned int counter = 0;
        bool center_is_valid = false;
        while(!center_is_valid) {
            counter++;
            x_new = rand_double(x_min,x_max);
            y_new = rand_double(y_min,y_max);
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
     * @brief Count the number of alive thermals at time t
     * @param {double} t; time
     */
    int nb_th_alive_at_time(double t) {
        int counter = 0;
        for(auto &th : thermals) {if(th->is_alive(t)) {++counter;}}
        return counter;
    }

    /**
     * @brief Compute the global environment sink rate
     * @param {double} z, t; altitude and time
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
     * @brief return ksi in range [w_star_min,w_star_max]
     * @note used for thermal creation
     */
    double pick_w_star() {return rand_double(w_star_min,w_star_max);}

    /**
     * @brief return zi in range [zi_min,zi_max]
     * @note used for thermal creation
     */
    double pick_zi() {//(double w_star) {
        //double noise = rand_double(-50.,50.);
        //return 961.0264191 * w_star - 701.9624694 + noise;
        return rand_double(zi_min,zi_max);
    }

    /**
     * @brief return lifespan as a function of w_star
     * @note used for thermal creation
     */
    double pick_lifespan(const double &w_star) {
        double a = (lifespan_max - lifespan_min) / (w_star_max - w_star_min);
        double b = lifespan_min - a * w_star_min;
        return a * w_star + b;
    }

    /**
     * @brief return ksi in range [ksi_min,ksi_max]
     * @note used for thermal creation
     */
    double pick_ksi() {return rand_double(ksi_min,ksi_max);}

    /** @brief Destructor */
    ~flat_thermal_soaring_zone() {
        for(auto th : thermals) {delete th;}
    }

    /** @brief Print the full scenario in the standard output stream */
    void print_scenario() {for(auto &th : thermals) {th->print_std_os();}}

    /** @brief Return the total number of thermals in the whole scenario */
    unsigned int get_total_nb_of_th() {return thermals.size();}

	/**
     * @brief Compute the wind velocity vector w at coordinate (x,y,z,t)
     * @param {double} x, y, z, t; coordinates
     * @param {std::vector<double> &} w; wind velocity vector [wx, wy, wz]
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
        if (!is_equal_to(noise_stddev,0.)) {
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator (seed);

            std::normal_distribution<double> distribution(0.,noise_stddev);
            //w[0] = distribution(generator);
            //w[1] = distribution(generator);
            w[2] = distribution(generator);
        }
        return *this;
    }

    /**
     * @brief Assert that the aircraft is inside the flight zone
     * @param {const double &} x, y, z; coordinates  in the earth frame
     * @return true if the input position belongs to the flight zone
     */
    virtual bool is_within_fz(const double &x, const double &y, const double &z) override {
        (void) z; //unused by default
        if (x_min<x && x<x_max && y_min<y && y<y_max) {return true;}
        return false;
    }

    /**
     * @brief Create a new thermal and push it back to 'thermals' vector attribute
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
     * @brief Create a scenario i.e. fill the 'thermals' vector attribute
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
     * @brief Write the updraft values in a file for visualization
     * @param {const double &} dx, dy; mesh precision
     * @param {const double &} z, t; altitude and time of the saved updraft field
     * @param {const std::string &} op; output path
     */
    void save_updraft_values(
        const double &dx,
        const double &dy,
        const double &z,
        const double &t,
        const std::string &op)
    {
        std::ofstream ofs;
        ofs.open(op);
        ofs << "t x y z updraft"<<std::endl;
        std::vector<double> w(3);
        for(double x=x_min; x<=x_max; x+=dx) {
            for(double y=y_min; y<=y_max; y+=dy) {
                wind(x,y,z,t,w);
                ofs << t << " ";
                ofs << x << " ";
                ofs << y << " ";
                ofs << z << " ";
                ofs << w.at(2);
                ofs << std::endl;
            }
        }
        ofs.close();
    }

    /**
     * @brief Save a scenario at the specified output path
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
     * @brief Save the flight zone dimensions
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

#endif
