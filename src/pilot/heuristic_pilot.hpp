#ifndef L2FSIM_HEURISTIC_PILOT_HPP_
#define L2FSIM_HEURISTIC_PILOT_HPP_

#include <pilot.hpp>
#include <beeler_glider/beeler_glider_state.hpp>
#include <beeler_glider/beeler_glider_command.hpp>

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
     * @param {double} arm; magnitude of the increment that one can apply to the angle
     * @param {double} kdalpha; coefficient for the D controller in alpha
     */
    double arm;
    double kdalpha;

    heuristic_pilot(
        double _angle_rate_magnitude = .1,
        double _kdalpha=.01) :
        arm(_angle_rate_magnitude),
        kdalpha(_kdalpha)
    {}

    /**
     * @brief Apply the policy
     * @param {state &} s; reference on the state
     * @param {command &} u; reference on the command
     * @warning dynamic cast
     * @note D-controller's coefficient is highly dependant on the configuration
     */
	pilot & operator()(state &_s, command &_u) override {
        beeler_glider_state &s = dynamic_cast <beeler_glider_state &> (_s);
        beeler_glider_command &u = dynamic_cast <beeler_glider_command &> (_u);

        // D-controller on alpha
        u.dalpha = kdalpha * (0. - s.gammadot);

        // No sideslip
        u.dbeta = 0.;

        // Increase/dicrease bank angle when lifted
        double sig = s.sigma;
        double mam = s.max_angle_magnitude;
        if(!is_less_than(s.zdot, .5)) { // lifted case zdot >= .5
            if(!is_less_than(sig, 0.)) { // sigma >= 0
                if (!is_greater_than(sig+arm, mam)) { // sigma + dsigma <= mam
                    u.dsigma = +arm;
                } else { // sigma + dsigma > mam
                    u.dsigma = 0.;
                }
            } else { // sigma < 0
                if (!is_less_than(sig-arm, -mam)) { // sigma - dsigma >= -mam
                    u.dsigma = -arm;
                } else { // sigma - dsigma < -mam
                    u.dsigma = 0.;
                }
            }
        } else { // no lift
            if (!is_less_than(sig, arm)) { // sigma >= arm
                u.dsigma = -arm;
            } else if(!is_greater_than(sig, -arm)) { //sigma <= -arm
                u.dsigma = +arm;
            } else { // -arm < sigma < arm
                u.dsigma = 0.;
            }
        }

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
        double mam = s.max_angle_magnitude;
        double x = s.x;
        double y = s.y;
        double khi = s.khi;
        double sig = s.sigma;
        double cs = -(x*cos(khi) + y*sin(khi)) / sqrt(x*x + y*y); // cos between heading and origin
        double th = .8; // threshold to steer back to flat command

        a.set_to_neutral();
        if (!is_less_than(sig,0.) && is_less_than(sig,mam)) {
            if (is_less_than(cs,th)) {
                if (is_less_than(sig+arm,mam)) {
                    a.dsigma = +arm;
                }
            } else {
                a.dsigma = -arm;
            }
        } else if (is_less_than(sig,0.) && is_less_than(-mam,sig)) {
            if (is_less_than(cs,th)) {
                if (is_less_than(-mam,sig-arm)) {
                    a.dsigma = -arm;
                }
            } else {
                a.dsigma = +arm;
            }
        }
		return *this;
    }
};

}

#endif
