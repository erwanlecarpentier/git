#ifndef L2FSIM_PASSIVE_PILOT_HPP_
#define L2FSIM_PASSIVE_PILOT_HPP_

#include <L2Fsim/pilot/pilot.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_state.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_command.hpp>

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
     * Attributes
     * @param {double} angle_rate_magnitude; magnitude of the increment that one can apply to the angle
     */
    double angle_rate_magnitude;

    passive_pilot(double _angle_rate_magnitude=.03) :
        angle_rate_magnitude(_angle_rate_magnitude)
    {}

    /**
     * Apply the policy
     * @param {state &} s; reference on the state
     * @param {command &} u; reference on the command
     * @warning dynamic cast
     * @note D-controller's coefficient is highly dependant on the configuration
     */
	pilot & operator()(state &_s, command &_u) override
	{
        (void) _s; // this is default
        beeler_glider_command &u = dynamic_cast <beeler_glider_command &> (_u);
        u.dalpha = 0.;
        u.dbeta = 0.;
        u.dsigma = 0.;
		return *this;
	}

    /**
     * Steer the glider back in the valid zone
     * @param {state &} _s; reference on the state
     * @param {command &} _u; reference on the command
     * @warning dynamic cast
     */
    pilot & out_of_boundaries(state &_s, command &_a) override {
        beeler_glider_state &s = dynamic_cast <beeler_glider_state &> (_s);
        beeler_glider_command &a = dynamic_cast <beeler_glider_command &> (_a);
        a.set_to_neutral();
        if(0. < s.sigma && s.sigma < .3) {
            a.dsigma = +angle_rate_magnitude;
        } else if (-.3 < s.sigma && s.sigma < 0.){
            a.dsigma = -angle_rate_magnitude;
        }
		return *this;
    }
};

}

#endif
