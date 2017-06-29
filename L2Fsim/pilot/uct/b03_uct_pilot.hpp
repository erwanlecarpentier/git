#ifndef L2FSIM_B03_UCT_PILOT_HPP_
#define L2FSIM_B03_UCT_PILOT_HPP_

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <L2Fsim/pilot/pilot.hpp>
#include <L2Fsim/pilot/uct/b03_node.hpp>
#include <L2Fsim/stepper/euler_integrator.hpp>

/**
 * @file b03_uct_pilot.hpp
 * @brief An online-anytime implementation of an omniscient UCT algorithm
 * @version 1.0
 * @since 1.0
 * @note compatibility: 'flat_thermal_soaring_zone.hpp'; 'beeler_glider.hpp'; 'beeler_glider_state.hpp'; 'beeler_glider_command.hpp'
 * @note make use of: 'b03_node.hpp'
 * @note the different actions available from a node's state are set via the method 'get_actions'
 * @note transition model is defined in function 'transition_model'
 * @note reward model is defined in function 'reward_model'
 * @note termination criterion for a node is set in 'is_terminal' method
 */

namespace L2Fsim{

class b03_uct_pilot : public pilot {
public:
    /**
     * @brief Attributes
     * @param {beeler_glider} ac; aircraft model
     * @param {flat_thermal_soaring_zone} fz; atmosphere model
     * @param {void (*transition_function)(aircraft &, flight_zone &, double &, const double &, const double &)}
     * @param {double} angle_rate_magnitude; magnitude of the increment that one can apply to the angles
     * @param {double} uct_parameter; parameter for the UCT formula
     * @param {double} time_step_width;
     * @param {double} sub_time_step_width;
     * @param {double} df; discount factor
     * @param {unsigned int} horizon; time limit for online simulations
     * @param {unsigned int} budget; number of expanded nodes in the tree
     */
    beeler_glider ac;
    flat_thermal_soaring_zone fz;
    void (*transition_function)(aircraft &, flight_zone &, double &, const double &, const double &);
    double angle_rate_magnitude;
    double uct_parameter;
    double time_step_width;
    double sub_time_step_width;
    double df;
    unsigned int horizon;
    unsigned int budget;

    /** @brief Constructor */
    b03_uct_pilot(
        beeler_glider &_ac,
        std::string sc_path,
        std::string envt_cfg_path,
        double noise_stddev,
        void (*_transition_function)(aircraft &, flight_zone &, double &, const double &, const double &),
        double _angle_rate_magnitude=.01,
        double _uct_parameter=1.,
        double _time_step_width=.1,
        double _sub_time_step_width=.1,
        double _df=.9,
        unsigned int _horizon=100,
        unsigned int _budget=1000) :
        ac(_ac),
        fz(sc_path, envt_cfg_path, noise_stddev),
        transition_function(_transition_function),
        angle_rate_magnitude(_angle_rate_magnitude),
        uct_parameter(_uct_parameter),
        time_step_width(_time_step_width),
        sub_time_step_width(_sub_time_step_width),
        df(_df),
        horizon(_horizon),
        budget(_budget)
    {}

    /**
     * @brief Get the value of dalpha with a D-controller in order to soften the phugoid behaviour
     * @param {beeler_glider_state &} s; current state
     * @return value of dalpha
     */
    double alpha_d_ctrl(const beeler_glider_state &s) {
        //double kd = .01, gammadot_ref=0.;
        //a.dalpha = .01 * (0. - s.gammadot);
        return - .01 * s.gammadot;
    }

    /**
     * @brief Get the available actions for a node, given its state
     * @param {const beeler_glider_state &} s; state of the node
     * @return {std::vector<beeler_glider_command>} vector of the available actions
     */
    std::vector<beeler_glider_command> get_actions(const beeler_glider_state &s) {
        std::vector<beeler_glider_command> v;
        double sig = s.sigma;
        double mam = s.max_angle_magnitude;
        double dalpha = alpha_d_ctrl(s);
        if(is_less_than(sig+angle_rate_magnitude, mam)) {
            v.emplace_back(dalpha,0.,angle_rate_magnitude);
        }
        if(is_less_than(-mam, sig-angle_rate_magnitude)) {
            v.emplace_back(dalpha,0.,-angle_rate_magnitude);
        }
        v.emplace_back(dalpha,0.,0.);
        return v;
    }

    /**
     * @brief Transition function model
     * @param {const beeler_glider_state &} s; current state
     * @param {const beeler_glider_command &} a; applied command
     * @return {beeler_glider_state} resulting state
     * @warning dynamic cast to beeler_glider_state
     */
    beeler_glider_state transition_model(const beeler_glider_state &s, const beeler_glider_command &a) {
        beeler_glider_state s_p = s;
        ac.set_state(s_p);
        ac.set_command(a);
        transition_function(ac,fz,s_p.time,time_step_width,sub_time_step_width);
        s_p = dynamic_cast <beeler_glider_state &> (ac.get_state()); // retrieve the computed state
        return s_p;
    }

    /**
     * @brief Reward function model
     * @param {const beeler_glider_state &} s; current state
     * @param {const beeler_glider_command &} a; applied command
     * @param {const beeler_glider_state &} sp; resulting state
     * @return {double} computed reward
     */
    double reward_model(
        const beeler_glider_state &s,
        const beeler_glider_command &a,
        const beeler_glider_state &sp)
    {
        (void)a; (void)sp; // unused by default
        double edot = s.zdot + s.V * s.Vdot / 9.81;
        return edot;
    }

    /**
     * @brief Pick randomly the indice of a not-expanded child
     * @param {const std::vector<unsigned int> &} nvis; vector of the children number of visit
     * @return A randomly picked indice corresponding to a child which has never been visited
     */
    unsigned int pick_new_child_indice(const std::vector<unsigned int> &nvis) {
        std::vector<unsigned int> ind;
        for(unsigned int i=0; i<nvis.size(); ++i) {
            if(nvis[i] == 0) {
                ind.push_back(i);
            }
        }
        return rand_element(ind);
    }

    /**
     * @brief Create a new child corresponding to an untried action
     * @param {b03_node &} v; parent node
     * @return {unsigned int} indice of the created child
     */
    unsigned int create_child(b03_node &v) {
        unsigned int indice = pick_new_child_indice(v.nb_visit);
        beeler_glider_state s_p = transition_model(v.s, v.actions.at(indice));
        v.children.at(indice) = b03_node(s_p, &v, get_actions(s_p), indice, v.depth+1, 0);
        return indice;
    }

    /**
     * @brief Compute the UCT score
     * @param {const double &} Qsa; Q value of the state-action pair
     * @param {double &} Ns; total number of visits
     * @param {double &} Nsa; number of visits of the state-action pair
     * @return UCT score
     */
    inline double uct_score(const double &Qsa, const double &Ns, const double &Nsa) {
        return Qsa + uct_parameter * sqrt(2. * log(Ns) / Nsa);
    }

    /**
     * @brief Get a reference on the 'best' child according to the UCT criteria
     * @param {const b03_node &} v; parent node
     * @return {b03_node} best UCT child
     */
    b03_node & best_uct_child(b03_node &v) {
        std::vector<double> scores;
        unsigned int Ns = v.total_number_of_visits;
        for(unsigned int i=0; i<v.children.size(); ++i) {
            scores.push_back(uct_score(v.Q_values[i], Ns, v.nb_visit[i]));
        }
        return v.children[argmax(scores)];
    }

    /**
     * @brief Boolean test for termination criterion
     * @param {beeler_glider_state &} s; reference on the tested state
     */
    bool is_terminal(beeler_glider_state &s) {
        return s.is_out_of_bounds();
    }

    /**
     * @brief Apply the tree policy from a 'current' node to a 'leaf' node; there are 3 cases:
     * 1. The current node is terminal: return a reference on the current node
     * 2. The current node is fully expanded: get the 'best' child according to UCT criteria and recursively run the function on this child
     * 3. The current node is not fully expanded: create a new child and return a reference on this new child
     * @param {b03_node &} v; current node of the tree exploration
     * @return {b03_node &} Reference on the resulting node
     * @note Recursive function
     */
    b03_node & tree_policy(b03_node &v0) {
        if (is_terminal(v0.s)) {
            return v0;
        } else if (v0.is_fully_expanded()) {
            return tree_policy(best_uct_child(v0));
        } else {
            return v0.children.at(create_child(v0));
        }
    }

    /**
     * @brief Run the default policy and get the reward
     * @param {const beeler_glider_state &} s; starting state
     * @param {double &} reward; computed reward
     */
    void default_policy(const beeler_glider_state &s, double &reward) {
        reward = 0.;
        beeler_glider_state s_tp, s_t=s;
        beeler_glider_command a_t;
        std::vector<beeler_glider_command> actions;
        for(unsigned int t=0; t<horizon; ++t) {
            actions = get_actions(s_t);
            a_t = rand_element(actions);
            s_tp = transition_model(s_t,a_t);
            reward += pow(df,(double)t) * reward_model(s_t,a_t,s_tp);
            if(is_terminal(s_tp)){break;}
            s_t = s_tp;
        }
    }

    /**
     * @brief Backup the reward computed via the default policy to the parents & update the number of visit counter
     * @param {b03_node &} v; node
     * @param {double &} reward;
     * @note recursive function
     */
    void backup(b03_node &v, const double &reward) { //TODO
        /*
        v.number_of_visits += 1;
        v.cumulative_reward += reward;
        // rather backup: reward_model((*v.parent).s,v.incoming_action,v.s) + df * reward;
        if(v.depth > 0) {
            backup(*v.parent, df * reward);
        }
        */
    }

    /**
     * @brief Get the action leading to the child with the highest average reward
     * @param {b03_node &} v; parent node
     * @param {beeler_glider_command &} a; resulting action
     */
    void best_action(b03_node &v, beeler_glider_command &a) {
        std::vector<double> scores;
        for(unsigned int i=0; i<v.children.size(); ++i) {
            scores.push_back(v.Q_values[i]);
        }
        a = v.actions[argmax(scores)];
    }

    /**
     * @brief Tree computation and action selection
     * @param {state &} _s; reference on the state
     * @param {command &} _a; reference on the command
     * @warning dynamic cast of state and action
     */
	pilot & operator()(state &_s, command &_a) override {
        beeler_glider_state &s0 = dynamic_cast <beeler_glider_state &> (_s);
        beeler_glider_command &a = dynamic_cast <beeler_glider_command &> (_a);
        b03_node v0(s0,nullptr,get_actions(s0),0,0,0); // root node
        for(unsigned int i=0; i<budget; ++i) {
            double reward = 0.;
            b03_node &v = tree_policy(v0);
            default_policy(v.s,reward);
            backup(v,reward);
        }
        best_action(v0,a);
        a.dalpha = alpha_d_ctrl(s0); // D-controller
        return *this;
	}

    /**
     * @brief Policy for 'out of boundaries' case
     * @param {state &} s; reference on the state
     * @param {command &} a; reference on the command
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
