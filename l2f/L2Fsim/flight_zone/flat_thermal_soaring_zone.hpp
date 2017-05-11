#ifndef L2FSIM_FLAT_THERMAL_SOARING_ZONE_HPP_
#define L2FSIM_FLAT_THERMAL_SOARING_ZONE_HPP_

#include <L2Fsim/flight_zone/flight_zone.hpp>
#include <L2Fsim/flight_zone/flat_zone.hpp>
#include <L2Fsim/flight_zone/thermal/std_thermal.hpp>
#include <string>
#include <fstream>
#include <sstream>

/**
 * @file flat_thermal_soaring_zone.hpp
 * @version 1.1
 * @since 1.0
 *
 * @brief This class implements the components of the wind vector by introducing the thermals in the flat zone
 * @note The abstract class flat_thermal_soaring_zone is a subclass of flat_zone
 */

namespace L2Fsim {

class flat_thermal_soaring_zone : public flat_zone
{
protected:
    /**
     * Attributes
     * @param {double} t_start; starting time of the simulation
     * @param {double} t_limit; ending time of the simulation
     * @param {double} windx, windy; horizontal components of the wind velocity
     * @param {double} w_star_min, w_star_max; minimum and maximum average updraft velocity
     * @param {double} zi_min, zi_max; minimum and maximum mixing layer thickness
     * @param {double} lifespan_min, lifespan_max; minimum and maximum lifespan
     * @param {double} x_min, x_max, y_min, y_max, z_min, z_max; boundaries
     * @param {double} ksi_min, ksi_max; minimum and maximum roll-off parameter
     * @param {double} d_min; minimum radius of a thermal
     * @param {std::vector<thermal *>} thermals; list of the thermals created in the simulation
     */
    double t_start=0.;
    double t_limit;
    double windx, windy;
    double w_star_min, w_star_max;
    double zi_min, zi_max;
    double lifespan_min, lifespan_max;
    double x_min, x_max, y_min, y_max, z_min, z_max;
    double ksi_min, ksi_max;
    double d_min;
    std::vector<thermal *> thermals;

    /** @brief Return the maximum number of thermals in the zone */
    int nbMaxThermals() {
        double zi_avg = zi_max - zi_min;
        return floor(0.6*(x_max-x_min)*(y_max-y_min)/(d_min*zi_avg));
    }

    /**
     * @brief Define the center of a new thermal depending on positions of other thermals
     * @note The new center is randomly picked within [x_min,x_max]*[y_min,y_max] and its validity is ensured
     * @param {double} t; time
     * @param {std::vector<double> &} center; computed center
     */
    bool create_thermal_center(double t, std::vector<double> &center) {
        double x_new, y_new;
        unsigned int counter = 0;
        bool center_is_valid = false;
        while(!center_is_valid) {
            counter++;
            x_new = rand_double(x_min,x_max);
            y_new = rand_double(y_min,y_max);
            std::vector<double> distances;
            for(auto& th:thermals) { // compute the distances to other thermals
                double x_th = th->get_center()[0];
                double y_th = th->get_center()[1];
                double dist_to_th = sqrt((x_new-x_th)*(x_new-x_th)+(y_new-y_th)*(y_new-y_th));
                distances.push_back(dist_to_th);
            }
            double minimum_distance = *std::min_element(distances.begin(),distances.end()); // #include <algorithm>
            if(minimum_distance>2.*d_min) {center_is_valid=true;}
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
        for(auto th: thermals) {if(th->is_alive(t)) {++counter;}}
        return counter;
    }

    /**
     * @brief Compute the global environment sink rate
     * @param {double} z, t; altitude and time
     */
    double global_sink_rate(double z, double t) {
        double thermals_area=0., mass_flow=0.;
        for(auto th: thermals) {
            if (th->is_alive(t)) {
                double z_zi = z / th->get_zi();
                double s_wd_th = ((.5 < (z_zi)) && ((z_zi) < .9)) ? 2.5*(z_zi - .5) : 0.;
                double avg_updraft_th = th->get_w_star() * pow(z_zi,1./3.) * (1. - 1.1*z_zi);
                double radius_th = .102 * pow((z_zi),1./3.) * (1 - .25*z_zi) * (z/z_zi);
                if(radius_th<10.){radius_th=10.;}
                double rsq = radius_th*radius_th;
                mass_flow += avg_updraft_th*M_PI*rsq*(1.-s_wd_th)*th->lifetime_coefficient(t);
                thermals_area += M_PI*rsq;
            }
        }
        double region_area = (x_max-x_min)*(y_max-y_min);
        double w_e = - mass_flow / (region_area - thermals_area);
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

public:
    /**
     * @brief Constructor; create an empty zone defined by its extremum dimensions
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
        double _d_min) :
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
        d_min(_d_min)
    {}

    /**
     * @brief Constructor; Read a saved thermal scenario in order to play an identical simulation of the flight zone
     * @param {std::string} filename; path to the saved scenario
     */
    flat_thermal_soaring_zone(std::string filename) {
        // TODO
        /*
        ifstream file(filename);
        if(!file){cerr << "Unable to open the file ("<< filename<<") read by FZ !" << std::endl;}

        std::string  line;
        std::vector<double> data;
        std::vector<std::vector<double>> datavector;
        int readline=1;
        while(getline(file, line))
        {
            istringstream stream(line);
            if(line[0]=='#')
                continue;
            switch(readline)
            {
                case 1: stream>>x_min>>x_max; break;
                case 2: stream>>y_min>>y_max; break;
                case 3: stream>>z_min>>z_max; break;
                case 4: stream>>t_start>>t_limit; break;
                case 5: stream>>windx>>windy; break;
                case 6: stream>>zi; break;
            }

            if(readline>6)
            {
                double th[]={0,0.0,0,0,0,0.0,0.0,0,0.0};
                stream>>th[0]>>th[1]>>th[2]>>th[3]>>th[4]>>th[5]>>th[6]>>th[7]>>th[8];
                data.push_back(th[0]);
                data.push_back(th[1]);
                data.push_back(th[2]);
                data.push_back(th[3]);
                data.push_back(th[4]);
                data.push_back(th[5]);
                data.push_back(th[6]);
                data.push_back(th[7]);
                data.push_back(th[8]);
                datavector.push_back(data);
                data.clear();
            }
            readline++;
        }
        file.close();
        for(auto data: datavector)
        {
            //TODO
            //std_thermal(mod,tB,XC0,YC0,ZC0,Zi,wstar,Lifetime,Ksi,read):
            std_thermal* newTH = new std_thermal(data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]);
            newTH->setwind(windx,windy);
            thermals.push_back(newTH);
        }
        std::cout<<"Configurations read file : "<<filename<<" ; and thermals are created accordingly."<<std::endl;
        */
    }

    /** Destructor */
    ~flat_thermal_soaring_zone() {
        for(auto th: thermals) {
            delete th;
        }
    }

	/**
     * @brief Compute the wind velocity vector w at coordinate (x,y,z,t)
     * @param {double} x, y, z, t; coordinates
     * @param {std::vector<double> &} w; wind velocity vector [wx, wy, wz]
     */
    flat_thermal_soaring_zone& wind(double x, double y, double z, double t, std::vector<double> &w) {
        std::vector<double>(3, 0.).swap(w);
        w[0]=windx; w[1]=windy; w[2]=0.;
        for(auto& th: thermals) {
            if(th->is_alive(t)) {
                th->wind(x,y,z,t,w);
            }
        }
        if(thermals.size()!=0 && thermals[0]->get_model()==1){
            w[2] += global_sink_rate(z,t);
        }
        return *this;
    }

    /**
     * @brief Create a new thermal
     * @param {int} model; thermal model
     * @param {double} t; current time, thermal's date of birth
     */
    void create_thermal(int model, double t) {
        std::vector<double> center;
        if(create_thermal_center(t,center)) {
            // previous code:
            //std_thermal(mod,tB,XC0,YC0,ZC0,Zi,wstar,Lifetime,Ksi,read):
            //std_thermal* new_th = new std_thermal(model,t,center[0],center[1],center[2],zi,0.,0,0.,0);
            double w_star = pick_w_star();
            double zi = pick_zi();
            double lifespan = pick_lifespan(w_star);
            double ksi = pick_ksi();
            std_thermal* new_th = new std_thermal(model,w_star,zi,t,lifespan,center.at(0),center.at(1),center.at(2),ksi);
            new_th->setwind(windx,windy);
            thermals.push_back(new_th);
        }
    }

    /**
     * @brief Create a scenario including thermals
     * @param {double} dt, refresh rate
     * @param {int} model; thermal model
     */
    void create_scenario(double dt, int model) {
        //TODO check what it does
        std::cout << "--> Create scenario" << std::endl;
        int max_nb_of_th = nbMaxThermals();
        for(double t=t_start; t<t_limit; t+=dt) {
            std::cout << "t = " << t << std::endl;
            while(nb_th_alive_at_time(t) < max_nb_of_th) {
                create_thermal(model,t);
            }
        }
        std::cout << std::endl;
    }

    /**
     * @brief Write the wind data for the visualization of a 'zslice' in a file
     * @param {double} deltaT, period of thermal actualization
     * @param {double} deltax, deltay; mesh precision
     * @param {double} zslice; height of the windfield
     * @param {std::string} filename; output path
     */
    void writeScenario(
        double deltaT,
        double deltax,
        double deltay,
        double zslice,
        std::string filename)
    {
        //TODO
        std::cout << "--> Write scenario" << std::endl;
        std::ofstream ofs;
        ofs.open(filename);

        ofs << "t x y z updraft"<<std::endl;

        for (double t=t_start; t<t_limit ; t+=deltaT) {
            std::cout << "t = " << t << std::endl;
            std::vector<double> w;
            for (int x=x_min; x<x_max; x+=deltax) {
                for (int y=y_min; y<y_max; y+=deltay) {
                    this->wind(x,y,zslice,t,w);
                    ofs << t << " " ;
                    ofs << x << " " ;
                    ofs << y << " " ;
                    ofs << zslice << " " ;
                    ofs << w[2];
                    ofs << std::endl;
                }
            }
        }
        ofs.close();
        std::cout << std::endl;
    }

    /**
     * @brief Save a scenario in '.cfg' format
     * @param {std::string} filename; output path
     */
    void save_configuration(std::string filename)
    {
        //TODO
        /*
        std::cout << "--> Save config"<< std::endl;
        std::ofstream ofs;
        ofs.open (filename);
        ofs<<"# Definition of the domain :\n# MinX and MaxX"<<std::endl
        <<x_min<<" "<<x_max<<std::endl
        <<"# MinY and MaxY"<<std::endl
        <<y_min<<" "<<y_max<<std::endl
        <<"# MinZ and MaxZ"<<std::endl
        <<z_min<<" "<<z_max<<std::endl
        <<"# t_start and t_limit"<<std::endl
        <<t_start<<" "<<t_limit<<std::endl
        <<"# windx and windy"<<std::endl
        <<windx<<" "<<windy<<std::endl
        <<"# zi "<<std::endl
        <<zi<<std::endl;

        ofs<<"# The list of Thermals created :"<<std::endl
        <<"# model   tBirth   CentreX   CentreY   CentreZ   zi   w_star   lifetime   ksi"<<std::endl;

        for(auto th: thermals) {
            ofs<<th->get_model()<< " " <<th->get_t_birth() << " " << th->get_center()[0]<<" "<<th->get_center()[1]<<" "<<th->get_center()[2]<<" "<<th->get_zi()<< " " << th->get_w_star() << " " << th->get_lifespan()  << " " << th->get_ksi()<< std::endl;
        }
        ofs.close();
        */
    }

    /**
     * @brief Save a scenario in '.csv' format
     * @param {std::string} filename; output path
     */
    void save_configuration_to_csv(std::string filename)
    {
        std::ofstream ofs;
        ofs.open(filename);
        ofs<<"model tBirth CentreX CentreY CentreZ zi w_star lifetime ksi"<<std::endl;

        for(auto th: thermals) {
            ofs<<th->get_model()<< " " <<th->get_t_birth() << " " << th->get_center()[0]<<" "<<th->get_center()[1]<<" "<<th->get_center()[2]<<" "<<th->get_zi()<< " " << th->get_w_star() << " " << th->get_lifespan()  << " " << th->get_ksi()<< std::endl;
        }
        ofs.close();
    }
};

}

#endif
