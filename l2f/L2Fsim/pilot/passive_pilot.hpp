#ifndef L2FSIM_PASSIVE_PILOT_HPP_
#define L2FSIM_PASSIVE_PILOT_HPP_

#include <L2Fsim/pilot/pilot.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_state.hpp>
#include <L2Fsim/aircraft/beeler_glider/beeler_glider_command.hpp>

/**
 * A passive pilot implementation compatible with the Beeler Glider model 'beeler_glider.hpp'
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
        beeler_glider_state &s = dynamic_cast <beeler_glider_state &> (_s);
        beeler_glider_command &u = dynamic_cast <beeler_glider_command &> (_u);
        double th = angle_rate_magnitude / 2.;
        double kd=1e-2, gammadot_ref=0.; // use kd=1e-3 for rk4 integrator

        // D-controller
        u.dalpha = kd * (gammadot_ref - s.gammadot);

        u.dbeta = 0.;
        if(s.sigma > +th) { u.dsigma = -angle_rate_magnitude; }
        if(s.sigma < -th) { u.dsigma = +angle_rate_magnitude; }

		return *this;
	}

    /**
     * Try to steer back the glider in the valid zone
     * @param {state &} s; reference on the state
     * @param {command &} u; reference on the command
     * @warning dynamic cast
     */
    pilot & out_of_range(state &_s, command &_u) override
    {
        beeler_glider_state &s = dynamic_cast <beeler_glider_state &> (_s);
        beeler_glider_command &u = dynamic_cast <beeler_glider_command &> (_u);
        u.dalpha = 0.;
        u.dbeta = 0.;
        if(s.sigma < 0.4) {
            u.dsigma = +angle_rate_magnitude;
        } else {
            u.dsigma = 0.;
        }
		return *this;
    }
};

}

#endif
