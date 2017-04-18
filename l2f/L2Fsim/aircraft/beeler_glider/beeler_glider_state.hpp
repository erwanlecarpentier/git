#ifndef L2FSIM_BEELER_GLIDER_STATE_HPP_
#define L2FSIM_BEELER_GLIDER_STATE_HPP_

//#include <L2Fsim/aircraft/state.hpp>

/**
 * Beeler's glider state from a Control point of view
 * @version 1.0
 * @since 1.0
 */

namespace L2Fsim {

class beeler_glider_state : public state {
public:
    /**
     * Attributes
     * State variables
     * @param {double} x, y, z; the absolute position in the earth frame
     * @param {double} gamma; elevation angle
     * @param {double} khi; azimuth angle
     * @param {double} alpha; angle of attack
     * @param {double} beta; sideslip angle
     * @param {double} sigma; bank angle
     * @param {double} xdot, ydot, zdot, Vdot, gammadot, khidot; rates
     * @param {double} time; current time (may be used by a planning module)
     */
    double x, y, z, V, gamma, khi;
    double alpha, beta, sigma;
    double xdot, ydot, zdot, Vdot, gammadot, khidot;
    double time;

    /** Constructor */
    beeler_glider_state(double _x=0.,
                        double _y=0.,
                        double _z=0.,
                        double _V=0.,
                        double _gamma=0.,
                        double _khi=0.,
                        double _alpha=0.,
                        double _beta=0.,
                        double _sigma=0.,
                        double _xdot=0.,
                        double _ydot=0.,
                        double _zdot=0.,
                        double _Vdot=0.,
                        double _gammadot=0.,
                        double _khidot=0.,
                        double _time=0.) :
        x(_x),
        y(_y),
        z(_z),
        V(_V),
        gamma(_gamma),
        khi(_khi),
        alpha(_alpha),
        beta(_beta),
        sigma(_sigma),
        xdot(_xdot),
        ydot(_ydot),
        zdot(_zdot),
        Vdot(_Vdot),
        gammadot(_gammadot),
        khidot(_khidot),
        time(_time)
    {}

    /**
     * Dynamically creates a copy of the state
     * @warning dynamic allocation: delete the duplicated object
     * @return a pointer to the copy
     */
    state * duplicate() const override {
        return new beeler_glider_state(*this);
    }

    void print() override {
        std::cout << "state.print: xyz=[";
        std::cout << x << " ";
        std::cout << y << " ";
        std::cout << z << "] V=";
        std::cout << V << " gamma=";
        std::cout << gamma << " khi=";
        std::cout << khi << " [";
        std::cout << alpha << " ";
        std::cout << beta << " ";
        std::cout << sigma << "] t=";
        std::cout << time << std::endl;
    }

    /** Set every dynamic variables to 0 */
    void clear_dynamic() override {
        xdot = 0.;
        ydot = 0.;
        zdot = 0.;
        Vdot = 0.;
        gammadot = 0.;
        khidot = 0.;
    }

    /**
     * Set the dynamic components i.e. the time derivatives interacting with the simulation integrator
     * @param {state &} s; state from which the dynamic components are copied
     * @warning dynamic cast from state to beeler_glider_state
     */
    void set_dynamic(state &_s) override {
        beeler_glider_state &s = dynamic_cast <beeler_glider_state &> (_s);
        xdot = s.xdot;
        ydot = s.ydot;
        zdot = s.zdot;
        Vdot = s.Vdot;
        gammadot = s.gammadot;
        khidot = s.khidot;
    }

    /**
     * Add the dynamic of a state to the current state
     * @param {state &} s; state from which the dynamic components are added
     * @param {const double} coef; a multiplicative coefficient
     * @warning dynamic cast from state to beeler_glider_state
     */
    void add_to_dynamic(state &_s, const double coef) override {
        beeler_glider_state &s = dynamic_cast <beeler_glider_state &> (_s);
        xdot += coef * s.xdot;
        ydot += coef * s.ydot;
        zdot += coef * s.zdot;
        Vdot += coef * s.Vdot;
        gammadot += coef * s.gammadot;
        khidot += coef * s.khidot;
    }

    /**
     * Apply a first order dynamic transition based on the values of the dynamic attributes (time derivatives)
     * @param {double} dt; time step
     */
    void apply_dynamic(const double dt) override {
        x += dt * xdot;
        y += dt * ydot;
        z += dt * zdot;
        V += dt * Vdot;
        gamma += dt * gammadot;
        khi += dt * khidot;
    }

    /**
     * Update time variable
     * @param {const double &} t; current time
     */
    void update_time(const double &t) override {
        time = t;
    }

    /**
     * Save (x, y, z, V, gamma, khi, alpha, beta, sigma, time) into a file
     * @param {std::string} filename; path of the log file
     */
    void save(std::string filename) override {
        std::ofstream output_file;
        double Edot = zdot + V*Vdot/9.81;
        if (time==0.){output_file.open(filename);}
        else{output_file.open(filename,std::ios::app);}

        std::string stream = "";
        stream += std::to_string(x) + " ";
        stream += std::to_string(y) + " ";
        stream += std::to_string(z) + " ";
        stream += std::to_string(V) + " ";
        stream += std::to_string(gamma) + " ";
        stream += std::to_string(khi) + " ";
        stream += std::to_string(alpha) + " ";
        stream += std::to_string(beta) + " ";
        stream += std::to_string(sigma) + " ";
        stream += std::to_string(Edot) + " ";
        stream += std::to_string(time);

        output_file << stream << std::endl;
        output_file.close();
    }
};

}

#endif
