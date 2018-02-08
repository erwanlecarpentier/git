#ifndef L2FSIM_BEELER_GLIDER_STATE_HPP_
#define L2FSIM_BEELER_GLIDER_STATE_HPP_

#include <ctgmath>

/**
 * @file beeler_glider_state.hpp
 * @brief Beeler's glider state from a Control point of view
 * @version 1.0
 * @since 1.0
 */

namespace L2Fsim {

class beeler_glider_state : public state {
public:
    /**
     * @brief Attributes
     * @param {double} x, y, z; the absolute position in the earth frame
     * @param {double} gamma; elevation angle
     * @param {double} khi; azimuth angle
     * @param {double} alpha; angle of attack
     * @param {double} beta; sideslip angle
     * @param {double} sigma; bank angle
     * @param {double} max_angle_magnitude; maximum angle magnitude
     * @param {double} xdot, ydot, zdot, Vdot, gammadot, khidot; rates
     * @param {double} time; current time
     */
    double x, y, z, V, gamma, khi;
    double alpha, beta, sigma;
    double max_angle_magnitude;
    double xdot, ydot, zdot, Vdot, gammadot, khidot;
    double time;

    /** @brief Constructor */
    beeler_glider_state(
        double _x=0.,
        double _y=0.,
        double _z=0.,
        double _V=0.,
        double _gamma=0.,
        double _khi=0.,
        double _alpha=0.,
        double _beta=0.,
        double _sigma=0.,
        double _max_angle_magnitude=.5,
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
        max_angle_magnitude(_max_angle_magnitude),
        Vdot(_Vdot),
        gammadot(_gammadot),
        khidot(_khidot),
        time(_time)
    {
        xdot = V * cos(khi) + cos(gamma);
        ydot = V * sin(khi) + cos(gamma);
        zdot = V * sin(gamma);
    }

    /** @brief Set time variable */
    void set_time(double t) override {time = t;}

    /** @brief Get x coordinate in the earth frame */
    double getx() {return x;}

    /** @brief Get y coordinate in the earth frame */
    double gety() {return y;}

    /** @brief Get z coordinate in the earth frame */
    double getz() {return z;}

    /** @brief Get time coordinate */
    double gett() {return time;}

    /**
     * @brief Dynamically creates a copy of the state
     * @warning dynamic allocation: delete the duplicated object
     * @return a pointer to the copy
     */
    state * duplicate() const override {
        return new beeler_glider_state(*this);
    }

    bool is_out_of_bounds() override {
        if (fabs(alpha) > max_angle_magnitude ||
            fabs(beta) > max_angle_magnitude ||
            fabs(sigma) > max_angle_magnitude ||
            z <= 0.)
        {return true;}
        return false;
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

    /** @brief Set every dynamic variables to 0 */
    void clear_dynamic() override {
        xdot = 0.;
        ydot = 0.;
        zdot = 0.;
        Vdot = 0.;
        gammadot = 0.;
        khidot = 0.;
    }

    /**
     * @brief Set the dynamic components i.e. the time derivatives interacting with the simulation integrator
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
     * @brief Add the dynamic of a state to the current state
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
     * @brief Apply a first order dynamic transition based on the values of the dynamic attributes (time derivatives)
     * @param {double} dt; time step
     */
    void apply_dynamic(const double dt) override {
        x += dt * xdot;
        y += dt * ydot;
        z += dt * zdot;
        V += dt * Vdot;
        gamma += dt * gammadot;
        gamma = acos(cos(gamma))*sign(sin(gamma));
        khi += dt * khidot;
        khi = acos(cos(khi))*sign(sin(khi));
    }

    /**
     * @brief Get a vector containing the saved variables
     * @return {std::vector<double>}
     */
    std::vector<double> get_save() override {
        return std::vector<double> {
            x,
            y,
            z,
            V,
            gamma,
            khi,
            alpha,
            beta,
            sigma,
            zdot + V*Vdot/9.81, // Edot
            time
        };
    }
};

}

#endif
