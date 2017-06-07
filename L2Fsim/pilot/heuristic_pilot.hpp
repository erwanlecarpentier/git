#ifndef L2FSIM_HEURISTIC_PILOT_HPP_
#define L2FSIM_HEURISTIC_PILOT_HPP_

#include <L2Fsim/pilot/pilot.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_state.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_command.hpp>

/**
 * @file heuristic_pilot.hpp
 * @brief A heuristic pilot implementation compatible with the Beeler Glider model 'beeler_glider.hpp'
 * @version 1.0
 * @since 1.0
 */

namespace L2Fsim {

class heuristic_pilot : public pilot {
public:
    /**
     * @brief Attributes
     * @param {double} angle_rate_magnitude; magnitude of the increment that one can apply to the angle
     */
    double angle_rate_magnitude;

    heuristic_pilot(double _angle_rate_magnitude=.03) :
        angle_rate_magnitude(_angle_rate_magnitude)
    {}

    /**
     * @brief Apply the policy
     * @param {state &} s; reference on the state
     * @param {command &} u; reference on the command
     * @warning dynamic cast
     * @note D-controller's coefficient is highly dependant on the configuration
     */
	pilot & operator()(state &_s, command &_u) override
	{
        beeler_glider_state &s = dynamic_cast <beeler_glider_state &> (_s);
        beeler_glider_command &u = dynamic_cast <beeler_glider_command &> (_u);
        double th = .5 * angle_rate_magnitude;
        double kd = 1e-2, gammadot_ref = 0.;

        // D-controller
        u.dalpha = kd * (gammadot_ref - s.gammadot);
        u.dbeta = 0.;
        if(s.sigma > +th) { u.dsigma = -angle_rate_magnitude; }
        if(s.sigma < -th) { u.dsigma = +angle_rate_magnitude; }

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
