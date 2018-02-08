#ifndef L2FSIM_STD_THERMAL_HPP_
#define L2FSIM_STD_THERMAL_HPP_

#include <thermal/thermal.hpp>
#include <utils.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>

/**
 * @file std_thermal.hpp
 * @version 1.1
 * @since 1.0
 * @brief The abstract class std_thermal is a subclass of thermal. It is a specialization of thermal.
 */

namespace L2Fsim {

class std_thermal : public thermal {
protected:
    /**
     * @brief Attributes
     * @param {int} model; thermal model selector
     * @param {double} t_birth; date of birth
     * @param {int} xc0, yc0, zc0; position of the thermal center at t_birth
     * @param {double} zi; convective mixing layer thickness [m]
     * @param {double} w_star; convective velocity scaling parameter [m/s]
     * @param {double} lifespan; life time
     * @param {double} ksi; shape factor linked to thermal life cycle
     */
    int model;
    double w_star;
    double zi;
    double t_birth;
    double lifespan;
    double xc0, yc0, zc0;
    double ksi;

public:
    /** @brief Constructor */
    std_thermal(
        int _model,
        double _w_star,
        double _zi,
        double _t_birth,
        double _lifespan,
        double _xc0,
        double _yc0,
        double _zc0,
        double _ksi) :
        model(_model),
        w_star(_w_star),
        zi(_zi),
        t_birth(_t_birth),
        lifespan(_lifespan),
        xc0(_xc0),
        yc0(_yc0),
        zc0(_zc0),
        ksi(_ksi)
    {}

    /** @brief Destructor */
    ~std_thermal() = default;

    double get_w_star() override {return w_star;}
    double get_t_birth() override {return t_birth;}
    double get_lifespan() override {return lifespan;}
    double get_zi() override {return zi;}
    int get_model() override {return model;}
    double get_ksi() override {return ksi;}

    void print() override {
        std::cout<<"model: "<<model<<" ";
        std::cout<<"t_birth: "<<t_birth<<" ";
        std::cout<<"lifespan: "<<lifespan<<" ";
        std::cout<<"w_star: "<<w_star<<" ";
        std::cout<<"zi: "<<zi<<" ";
        std::cout<<"center: "<<xc0<<" "<<yc0<<" "<<zc0<<" ";
        std::cout<<"ksi: "<<ksi<<" ";
        std::cout<<std::endl;
    }

    /**
     * @brief Get center coordinate in the earth frame
     * @return {std::vector<double>} 3D vector containing the position of the center
     */
    std::vector<double> get_center() override {
        std::vector<double> w;
        w.push_back(xc0);
        w.push_back(yc0);
        w.push_back(zc0);
        return w;
    }

    /**
     * @brief Calculate the distance between the given point(x,y,z) and the thermals center
     * @note Effect of ambient winds and thermal drifting is considered
     */
    double dist_to_updraft_center(const double x, const double y, const double z) override {
        double xcz = xc0 + windx*z; // drifted center at alttitude z
        double ycz = yc0 + windy*z; // drifted center at alttitude z
        return sqrt((xcz-x)*(xcz-x) + (ycz-y)*(ycz-y));
    }

    /**
     * @brief Return true if the thermal is alive, else false
     * @param {const double} t; current time
     */
    bool is_alive(const double t) override {
        return ((t_birth<=t) && (t<=(t_birth+lifespan)))?true:false;
    }

    /**
     * @brief Return the thermal life cycle coefficient
     * @param {const double} t; current time
     */
    double lifetime_coefficient(const double t) override
    {
        double abstau = fabs(t-t_birth-lifespan/2.);
        double T = (1.+ksi) / lifespan;
        double D = (1.-ksi) / (2.*T);
        double c_t = 0.;
        if (abstau <= D) {c_t = 1.;}
        else if(abstau <= (1.+ksi)/(2.*T)) {
            c_t = .5*(1.+cos(M_PI*T*(abstau-D)/ksi));
        }
        return c_t;
    }

    /**
     * @brief Allen's thermal model
     * @param {const double} r, z, t; radius, altitude and time
     * @note The time normally does not have an influence in the Allen model; However, here we compute the lifetime coefficient inside the 'allen_model' method in order to optimize the code
	 */
    double allen_model(const double r, const double z, const double t)
    {
        double z_zi = z/zi;
        double r2 = std::max(10.,.102*pow(z_zi,1./3.)*(1.-.25*z_zi)*zi);
        if(r > 2.*r2) {
            return 0.;
        } else {
            //double r1_r2 = .36;
            double r1 = .36*r2;
            double r_r2 = r/r2;
            double w_ = w_star * pow(z_zi,1./3.) * (1. - 1.1*z_zi);
            double w_peak = 3. * w_ * (r2-r1)*r2*r2 / (r2*r2*r2 - r1*r1*r1);
            double w_l = ((r1 < r) && (r < (2.*r2))) ? -M_PI/6.*sin(M_PI*r_r2) : 0.;
            double s_wd = ((.5 < z_zi) && (z_zi < .9)) ? 2.5*(z_zi-.5) : 0.;
            //double w_d = s_wd*w_l;
            //double k1 = 1.4866; // ki values valid for r1/r2 = 0.36
            //double k2 = 4.8354;
            //double k3 = -.0320;
            //double k4 = .0001;
            return lifetime_coefficient(t) * w_peak * (1./(1.+pow(fabs(1.4866*r_r2 - .0320),4.8354)) + .0001*r_r2 + s_wd*w_l);
        }
    }

    /**
     * @brief Childress's thermal model
     * @param {const double} r, z; radius and altitude
     * @ref An Empirical Model of thermal Updrafts Using Data Obtained From a Manned Glider, Christopher E. Childress
	 */
    double childress_model(const double r, const double z)
    {
        double w_total = 0.;
        if(z>zi) {w_total=0.;} // flight level higher than CBL
        else {
            double z_zi = z/zi;

            //Calculation of radius of the thermal
            double d_T = zi*(.4 * pow(z_zi,1./3.) * (1 - .5*z_zi)) + (z*z_zi*z_zi - .6*z*z_zi)/M_PI;
            double r2 = .5*d_T;

            //Core downdraft radius
            double r1 = .5*(.17*d_T + .5*(z_zi - .6)*d_T);

            // Calculating w_ and w_peak based on Allen model
            double w_ = w_star * pow((z / zi),1./3.) * (1 - 1.1*z_zi);
            double w_peak = 3.*w_*(r2-r1)*r2*r2 / (r2*r2*r2-r1*r1*r1);

            // Calculation of downdraft terms
            double w_dec = (-zi/(1.275*w_star*w_star))*(12.192/z);
            double wd = w_dec*(z_zi + .45) - .5*w_peak;

            //Calculation of Updraft based on equations 14,15,16 from Childress
            if(z_zi<.5 && r<=r2) {
                w_total = w_peak*cos((r/r2)*M_PI*.5);
            }
            else if(z_zi<.9) {
                if(r<r1) {
                    w_total = wd*cos((r/r1)*M_PI*.5);
                }
                else if(r1<=r && r<=(r1+r2)) {
                    w_total = w_peak*sin((r-r1)/r2 * 1.212 *M_PI);
                    if(w_total<0.){w_total=0.;}
                }
                else {w_total = 0.;}
            }
            else {
                if(r<r1) {w_total = .5*wd*cos((r/r1)*M_PI*.5);}
                else if(r1<=r && r<=(r1+r2)) {
                    w_total = (1-z_zi)*w_peak*sin((r-r1)/r2 * 1.212 *M_PI);
                    if(w_total<0.){w_total=0.;}
                }
                else {w_total = 0.;}
            }
        }
        return w_total;
    }

    /**
     * @brief Lenschow's thermal model
     * @param {const double} r, z; radius and altitude
     * @param {const bool} choice; 1: with Gaussian distribution; 2: with Geodon model
	 */
    double lenschow_model(const double r, const double z, const bool choice)
    {
        double w_total = 0.;
        if(z>zi) {w_total=0.;} // flight level higher than CBL
        else {
            double z_zi = z/zi;
            double z_zi_powthird = pow(z_zi,1./3.);
            // diameter of the thermal given by
            double d = 0.16 * z_zi_powthird * (1 - .25*z_zi) * zi;

            //normalized updraft velocity
            double w_= w_star * z_zi_powthird * (1 - 1.1*z_zi);

            // variance of the updraft velocity
            //double var = 1.8*pow(z_zi,2./3.) * (1.-.8*z_zi)*(1.-.8*z_zi);

            // w_peak assuming a gaussian distribution is
            double w_peak = w_; //*(2./pow(M_PI,.5));

            if (choice == 1) {
                w_total = w_peak * exp(-(4.*r*r/(d*d)));
            }
            else {
                w_total = w_peak * exp(-(4.*r*r/(d*d)))*(1.-(4.*r*r/(d*d)));
            }
        }
        return w_total;
    }

    double integral_wz_allen(double h)
    {
        if (h>1100.) {h=1100.;}
        return 1./(2.64 * pow((h/1400.),1./3.) * (1. - 1.1*h/1400.));
    }

    double simpsons(double (*f)(double x), const double a, const double b, const int n)
    {
        double h = (b-a) / (double)n;
        double x=0., r=0., s=0.;
        char m = 0;
        for(x=a; x<=b; x+=h) {
            r = f(x);
            if (x == a || x == b) {
                s += r;
            }
            else {
                m = !m;
                s += r * (m+1) * 2.;
            }
        }
        return s * h / 3.;
    }

    /**
     * @brief Lawrance's thermal model
     * @param {std::vector<double> &} w; wind vector
     * @param {const double} x, y, z, t; spatio-temporal coordinates
	 */
    void lawrance_model(
        std::vector<double> &w,
        const double x,
        const double y,
        const double z,
        const double t)
    {
        (void) t; // Unused by default
        double r1_rT = .36;
        double k = 3.;
        double z_zi = z/zi;
        double z_zi_powthird = pow(z_zi,1./3.);

        double rT = std::max(10., .102 * z_zi_powthird * (1.-.25*z_zi) * zi);
        double r1 = r1_rT * rT;

        //Calculating w_ and W_peak using
        double w_ = w_star * z_zi_powthird * (1.-1.1*z_zi);
        double w_core = 3.*w_*(rT-r1)*rT*rT / (rT*rT*rT - r1*r1*r1);

        double x0=0., y0=0., z0=800.;
        if(z0<k*rT) { // The bubble has not detached from the ground yet
            x0 = xc0;
            y0 = yc0;
        }
        else { // The bubble is completely formed and it can detach itself from the ground and move along with the wind
            x0 = xc0; //+ simpsons(integral_wz_allen(z),100.0,z,1000)*windx;
            y0 = yc0; //+ simpsons(integral_wz_allen(z),100.0,z,1000)*windy;
        }

        double xt = x-x0;
        double yt = y-y0;
        double zt = z-z0;
        double dH = sqrt(xt*xt + yt*yt);

        //Calculation of Wz
        if(dH == 0.) {
            w[2] += w_core * .5*(cos(M_PI*zt/(k*rT))+1.);
        }
        else if(dH <= 2.*rT) {
            w[2] += .5*(w_core*rT)/(M_PI*dH) * sin(M_PI*dH/rT) * (cos(M_PI*zt/(k*rT))+1.);
        }
        else {/*w[2] += 0.;*/}

        //if(fabs(zt)>(k*rT)) {w[2] += 0.;}

        //calculation of windx and windy
        if(dH!=0. || dH<rT) {
            double coef = (w[2]*zt)/(dH*(dH-rT)*k*k);
            w[0] -= coef*xt;
            w[1] -= coef*yt;
        }
        else if(dH == rT) {
            double dw = w_core/(2.*k*rT) * (1. + cos(M_PI*z/(k*rT)));
            w[0] -= dw;
            w[1] -= dw;
        }
        /*else {
            w[0]+=0.;
            w[1]+=0.;
        }*/
    }

    std_thermal& wind(const double x, const double y, const double z, const double t, std::vector<double> &w) override
    {
        if (z>zi || z<zc0) {w[2]=0.;}
        else {
            double r = dist_to_updraft_center(x,y,z);
            switch(model) {
                case 1: { // Allen model
                    w[2] += allen_model(r,z,t);
                    break;
                }
                case 2: { // Childress model
                    w[2] += childress_model(r,z)*lifetime_coefficient(t);
                    break;
                }
                case 3: { // Lenschow with Gaussian distribution
                    w[2] += lenschow_model(r,z,1)*lifetime_coefficient(t);
                    break;
                }
                case 4: { // Lenschow with Geodon model
                    w[2] += lenschow_model(r,z,0)*lifetime_coefficient(t);
                    break;
                }
                case 5: { // Lawrance model
                    lawrance_model(w,x,y,z,t);
                    double c_t = lifetime_coefficient(t);
                    w[0] *= c_t;
                    w[1] *= c_t;
                    w[2] *= c_t;
                    break;
                }
                default: {
                    std::cout << "Warning: chosen thermal model unavailable" << std::endl;
                    std::cout << "Method 'wind' in std_thermal unable to compute updraft" << std::endl;
                }
            }
        }
        return *this;
    }
};

}

#endif
