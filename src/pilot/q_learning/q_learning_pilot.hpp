#ifndef L2FSIM_Q_LEARNING_PILOT_HPP_
#define L2FSIM_Q_LEARNING_PILOT_HPP_

#include <cstdio>
#include <cstdlib>
#include <pilot.hpp>

/**
 * @brief An online implementation of a Q-Learning algorithm
 * @version 1.0
 * @since 1.0
 *
 * @note compatibility: 'beeler_glider.hpp'; 'beeler_glider_state.hpp'; 'beeler_glider_command.hpp'
 * @note action space defined in method 'get_available_actions'
 * @note feature vector defined in method 'get_feature_vector'
 * @note reward function defined in method 'get_reward'
 */

namespace L2Fsim{

class q_learning_pilot : public pilot {
public:
    /**
     * @brief Attributes
     * @param {beeler_glider_state} prev_s; previous state
     * @param {beeler_glider_command} prev_a; previous command
     * @param {double} angle_rate_magnitude; magnitude of the increment that one can apply to the angles
     * @param {double} kdalpha; coefficient for the D controller in alpha
     * @param {double} epsilon; for epsilon-greedy policy
     * @param {double} lr; learning rate (aka alpha in the literature)
     * @param {double} df; discount factor (aka gamma in the literature)
     * @param {std::vector<double>} parameters; parameters vector
     */
    beeler_glider_state prev_s;
    beeler_glider_command prev_a;
    double angle_rate_magnitude;
    double kdalpha;
    double epsilon;
    double lr;
    double df;
    std::vector<double> parameters;

    q_learning_pilot(
        double _angle_rate_magnitude=.0,
        double _kdalpha=.01,
        double _epsilon=.01,
        double _lr=.01,
        double _df=.9) :
        prev_s(),
        prev_a(),
        angle_rate_magnitude(_angle_rate_magnitude),
        kdalpha(_kdalpha),
        epsilon(_epsilon),
        lr(_lr),
        df(_df)
    {
        std::vector<double> phi = get_feature_vector(prev_s,prev_a);
        parameters.resize(phi.size(),0.);
    }

    /**
     * @brief Normalizing function, sigmoid-like
     * @param {const double} x; quantity wished to be maximised
     * @param {const double} x_max; maximum magnitude
     * @return {double} normalized quantity scaled in ]-1,1[ interval
     */
    double scale(const double x, const double x_max) {
        return 2. * sigmoid(x,x_max,0.) - 1.;
    }

    /**
     * @brief Evaluate the feature vector at (s, a)
     * @param {const beeler_glider_state &} s; state
     * @param {const beeler_glider_command &} a; action
     * @return {std::vector<double>} feature vector
     */
    std::vector<double> get_feature_vector(
        const beeler_glider_state &s,
        const beeler_glider_command &a)
    {
        std::vector<double> phi = {
            1.,
            s.zdot/20.,//scale(s.zdot,10.),
            s.gammadot/.1,//scale(s.gammadot,.2),
            s.sigma/s.max_angle_magnitude,//scale(s.sigma,s.max_angle_magnitude),
            a.dsigma / angle_rate_magnitude
        };
        unsigned int sz = phi.size();
        for(unsigned int i=1; i<sz; ++i) {
            for(unsigned int j=i; j<sz; ++j) {
                phi.push_back(phi[i]*phi[j]);
            }
        }
        return phi;
    }

    /**
     * @brief Evaluate the Q function
     * @param {const beeler_glider_state &} s; state input
     * @param {const beeler_glider_command &} a; command input
     * @return {double} Q value
     */
    double q_value(const beeler_glider_state &s, const beeler_glider_command &a)
    {
        double score = 0.;
        std::vector<double> phi = get_feature_vector(s,a);
        for(unsigned int i=0; i<phi.size(); ++i) {
            score += phi.at(i) * parameters.at(i);
        }
        return score;
    }

    /**
     * @brief Get the available actions from the current state
     * @param {const beeler_glider_state &} s; state
     * @return {std::vector<beeler_glider_command>} vector of the available actions
     */
    std::vector<beeler_glider_command> get_avail_actions(const beeler_glider_state &s)
    {
        std::vector<beeler_glider_command> v;
        double sig = s.sigma;
        double mam = s.max_angle_magnitude;
        if(is_less_than(sig+angle_rate_magnitude, +mam)) {
            v.push_back(beeler_glider_command(0.,0.,+angle_rate_magnitude));
        }
        if(is_less_than(-mam, sig-angle_rate_magnitude)) {
            v.push_back(beeler_glider_command(0.,0.,-angle_rate_magnitude));
        }
        v.push_back(beeler_glider_command(0.,0.,0.));
        return v;
    }

    /**
     * @brief Set the value of an input command accordingly to an epsilon-greedy policy given an input state
     * @param {const beeler_glider_state &} s; input state
     * @param {beeler_glider_command &} a; modified command
     */
    void epsilon_greedy_policy(const beeler_glider_state &s, beeler_glider_command &a)
    {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> distribution(0.,1.);

        std::vector<beeler_glider_command> aa = get_avail_actions(s);
        std::vector<unsigned int> max_ind, non_max_ind;
        std::vector<double> scores;

        for(auto &action : aa) {
            scores.push_back(q_value(s,action));
        }
        sort_indices(scores,max_ind,non_max_ind);

        if(distribution(generator) > epsilon) { // Greedy action
            a = aa.at(rand_element(max_ind));
        } else { // Random action
            if(non_max_ind.size() == 0) { // scores are all the same
                a = aa.at(rand_element(max_ind));
            } else {
                a = aa.at(rand_element(non_max_ind));
            }
        }
    }

    /**
     * @brief Greedy policy a = argmax Q(s,.)
     * @param {const beeler_glider_state &} s; input state
     * @param {beeler_glider_command &} a; modified command
     * @note Ties are broken randomly
     */
    void greedy_policy(const beeler_glider_state &s, beeler_glider_command &a)
    {
        std::vector<beeler_glider_command> aa = get_avail_actions(s);
        std::vector<unsigned int> max_ind, non_max_ind;
        std::vector<double> scores;

        for(unsigned int i=0; i<aa.size(); ++i) {
            scores.push_back(q_value(s,aa.at(i)));
        }
        sort_indices(scores,max_ind,non_max_ind);
        a = aa.at(rand_element(max_ind));
    }

    /**
     * @brief Reward function
     * @param {const beeler_glider_state &} s; state t
     * @param {const beeler_glider_command &} a; action t
     * @param {const beeler_glider_state &} s_p; state t+1
     * @param {double &} reward; reward r(s, a, s_p)
     */
    void get_reward(
        const beeler_glider_state &s,
        const beeler_glider_command &a,
        const beeler_glider_state &s_p,
        double &reward)
    {
        (void) a; (void) s_p; // unused by default
        double edot = s.zdot + s.V * s.Vdot / 9.81;
        reward = edot/10.;//scale(edot,10.);
    }

    /**
     * @brief Update the parameters vector
     * @param {const beeler_glider_state &} s; state of the update
     * @param {const beeler_glider_command &} a; action of the update
     * @param {const double} delta; cf q-learning update equation
     */
    void update_parameters(
        const beeler_glider_state &s,
        const beeler_glider_command &a,
        const double delta)
    {
        std::vector<double> phi = get_feature_vector(s,a);
        for (unsigned int i=0; i<parameters.size(); ++i) {
            parameters.at(i) += lr * delta * phi.at(i);
        }
    }

    /**
     * @brief An online Q-Learning algorithm step and a command control are performed at each time step of the simulation
     * @param {state &} s; reference on the state
     * @param {command &} a; reference on the command
     * @warning dynamic cast of state and action
     */
	pilot & operator()(state &_s, command &_a) override
	{
        beeler_glider_state &s = dynamic_cast <beeler_glider_state &> (_s);
        beeler_glider_command &a = dynamic_cast <beeler_glider_command &> (_a);
        beeler_glider_command a_off_policy;
        double reward = 0.;

        get_reward(prev_s, prev_a, s, reward); // r(s, a, s_p)
        greedy_policy(s, a_off_policy);
        double delta = reward + df * q_value(s, a_off_policy) - q_value(prev_s, prev_a);
        update_parameters(prev_s, prev_a, delta);

        epsilon_greedy_policy(s, a);
        a.dalpha = kdalpha * (0. - s.gammadot);
        prev_s = s;
        prev_a = a;

		return *this;
	}

    /**
     * @brief Policy for 'out of range' situations
     * @param {state &} s; reference on the state
     * @param {command &} a; reference on the command
     */
    pilot & out_of_boundaries(state &_s, command &_a) override
    {
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
