#ifndef L2FSIM_PASSIVE_PILOT_HPP_
#define L2FSIM_PASSIVE_PILOT_HPP_

#include <pilot.hpp>
#include <beeler_glider/beeler_glider_state.hpp>
#include <beeler_glider/beeler_glider_command.hpp>

/**
 * @file passive_pilot.hpp
 * @brief A passive pilot implementation compatible with the Beeler Glider model 'beeler_glider.hpp'
 * @version 1.1
 * @since 1.0
 *
 * The pilot takes the state vector as input and return a neutral command
 */

namespace L2Fsim {

class passive_pilot : public pilot {
public:
    /**
     * @brief Attributes
     * @param {double} angle_rate_magnitude; magnitude of the increment that one can apply to the angle
     */
    double angle_rate_magnitude;

    /** @brief Constructor */
    passive_pilot(double _angle_rate_magnitude = .1) :
        angle_rate_magnitude(_angle_rate_magnitude)
    {}

    /** @brief Destructor */
    ~passive_pilot() = default;

    /**
     * @brief Apply the policy
     * @param {state &} s; reference on the state
     * @param {command &} u; reference on the command
     * @warning dynamic cast
     * @note D-controller's coefficient is highly dependant on the configuration
     */
	pilot & operator()(state &_s, command &_u) override {
        (void) _s; // this is default
        beeler_glider_command &u = dynamic_cast <beeler_glider_command &> (_u);
        u.dalpha = 0.;
        u.dbeta = 0.;
        u.dsigma = 0.;
		return *this;
	}

    /**
     * @brief Steer the glider back in the valid zone
     * @param {state &} _s; reference on the state
     * @param {command &} _u; reference on the command
     * @warning dynamic cast
     */
    pilot & out_of_boundaries(state &_s, command &_a) override {
        beeler_glider_state &s = dynamic_cast <beeler_glider_state &> (_s);
        beeler_glider_command &a = dynamic_cast <beeler_glider_command &> (_a);
        double ang_max = .4;
        double x = s.x;
        double y = s.y;
        double khi = s.khi;
        double sigma = s.sigma;
        double cs = -(x*cos(khi) + y*sin(khi)) / sqrt(x*x + y*y); // cos between heading and origin
        double th = .8; // threshold to steer back to flat command

        a.set_to_neutral();
        if (!is_less_than(sigma,0.) && is_less_than(sigma,ang_max)) {
            if (is_less_than(cs,th)) {
                if (is_less_than(sigma+angle_rate_magnitude,ang_max)) {
                    a.dsigma = +angle_rate_magnitude;
                }
            } else {
                a.dsigma = -angle_rate_magnitude;
            }
        } else if (is_less_than(sigma,0.) && is_less_than(-ang_max,sigma)) {
            if (is_less_than(cs,th)) {
                if (is_less_than(-ang_max,sigma-angle_rate_magnitude)) {
                    a.dsigma = -angle_rate_magnitude;
                }
            } else {
                a.dsigma = +angle_rate_magnitude;
            }
        }
		return *this;
    }
};

}

#endif
