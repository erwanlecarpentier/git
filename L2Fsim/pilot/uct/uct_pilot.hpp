#ifndef L2FSIM_UCT_PILOT_HPP_
#define L2FSIM_UCT_PILOT_HPP_

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <L2Fsim/pilot/pilot.hpp>
#include <L2Fsim/pilot/uct/uct_node.hpp>
#include <L2Fsim/stepper/euler_integrator.hpp>

/**
 * @file uct_pilot.hpp
 * @brief An online-anytime implementation of a deterministic UCT algorithm
 * @version 1.1
 * @since 1.0
 * @note Compatibility: 'flat_thermal_soaring_zone.hpp'; 'beeler_glider.hpp'; 'beeler_glider_state.hpp'; 'beeler_glider_command.hpp'
 * @note Make use of: 'uct_node.hpp'
 * @note The different actions available from a node's state are set via the method 'get_actions'
 * @note Transition model is defined in method 'transition_model'
 * @note Reward model is defined in method 'reward_model'
 */

namespace L2Fsim{

class uct_pilot : public pilot {
public:
    /**
     * @brief Attributes
     * @param {beeler_glider} ac; aircraft model
     * @param {flat_thermal_soaring_zone} fz; atmosphere model
     * @param {void (*transition_function)(aircraft &, flight_zone &, double &, const double &, const double &)}
     * @param {double} angle_rate_magnitude; magnitude of the increment that one can apply to the angles
     * @param {double} kdalpha; coefficient for the D controller in alpha
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
    double kdalpha;
    double uct_parameter;
    double time_step_width;
    double sub_time_step_width;
    double df;
    unsigned int horizon;
    unsigned int budget;
    unsigned int default_policy_selector;

    /** @brief Constructor */
    uct_pilot(
        beeler_glider &_ac,
        std::string sc_path,
        std::string envt_cfg_path,
        double noise_stddev,
        void (*_transition_function)(aircraft &, flight_zone &, double &, const double &, const double &),
        double _angle_rate_magnitude=.01,
        double _kdalpha=.01,
        double _uct_parameter=1.,
        double _time_step_width=.1,
        double _sub_time_step_width=.1,
        double _df=.9,
        unsigned int _horizon=100,
        unsigned int _budget=1000,
        unsigned int _default_policy_selector=0) :
        ac(_ac),
        fz(sc_path, envt_cfg_path, noise_stddev),
        transition_function(_transition_function),
        angle_rate_magnitude(_angle_rate_magnitude),
        kdalpha(_kdalpha),
        uct_parameter(_uct_parameter),
        time_step_width(_time_step_width),
        sub_time_step_width(_sub_time_step_width),
        df(_df),
        horizon(_horizon),
        budget(_budget),
        default_policy_selector(_default_policy_selector)
    {}

    /**
     * @brief Get the value of dalpha with a D-controller in order to soften the phugoid behaviour
     * @param {beeler_glider_state &} s; current state
     * @return value of dalpha
     */
    double alpha_d_ctrl(const beeler_glider_state &s) {
        //a.dalpha = kdalpha * (0. - s.gammadot);
        return - kdalpha * s.gammadot;
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
        std::random_shuffle(v.begin(), v.end());
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
        return sigmoid(edot,10.,0.);
    }

    /**
     * @brief Create a new child corresponding to an untried action
     * @param {uct_node &} v; parent node
     * @return {unsigned int} indice of the created child
     */
    unsigned int create_child(uct_node &v) {
        unsigned int indice = v.children.size();
        beeler_glider_state s_p = transition_model(v.s, v.actions.at(indice));
        v.children.emplace_back(s_p, &v, get_actions(s_p), indice, v.depth+1);
        v.rewards.at(indice) = reward_model(v.s,v.actions.at(indice),s_p); // save the transition reward
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
     * @param {const uct_node &} v; parent node
     * @return {uct_node} best UCT child
     */
    uct_node & best_uct_child(uct_node &v) {
        std::vector<double> scores;
        unsigned int Ns = v.total_nb_visits;
        for(unsigned int i=0; i<v.children.size(); ++i) {
            scores.push_back(uct_score(v.Q_values[i], Ns, v.nb_visits[i]));
        }
        return v.children[argmax(scores)];
    }

    /**
     * @brief Apply the tree policy from a 'current' node to a 'leaf' node; there are 3 cases:
     * 1. The current node is terminal: return a reference on the current node
     * 2. The current node is fully expanded: get the 'best' child according to UCT criteria and recursively run the method on this child
     * 3. The current node is not fully expanded: create a new child and return a reference on this new child
     * @param {uct_node &} v; current node of the tree exploration
     * @return {uct_node &} Reference on the resulting node
     * @note Recursive method
     */
    uct_node & tree_policy(uct_node &v) {
        if (v.is_terminal()) {
            return v;
        } else if (v.is_fully_expanded()) {
            return tree_policy(best_uct_child(v));
        } else {
            return v.children.at(create_child(v));
        }
    }

    /**
     * @brief Select an action among the available ones, there are two cases:
     * - lifted case (zdot >= 0); increase the magnitude of sigma
     * - no lift case (zdot < 0); dicrease the magnitude of sigma
     * @param {const std::vector<beeler_glider_command> &} actions; available actions
     * @param {beeler_glider_state &} s; state
     * @param {beeler_glider_command &} a; chosen action
     * @return Indice of the chosen action
     */
    unsigned int heuristic_policy(
        const std::vector<beeler_glider_command> &actions,
        beeler_glider_state &s,
        beeler_glider_command &a)
    {
        double sig = s.sigma;
        if(!is_less_than(s.zdot, 0.)) { // lifted case, zdot >= 0
            if(!is_less_than(sig, 0.)) { // sigma >= 0; select +dsigma or 0
                for(unsigned int i=0; i<actions.size(); ++i) { // select +dsigma
                    if(is_greater_than(actions[i].dsigma, 0.)) {
                        a = actions[i];
                        return i;
                    }
                }
                for(unsigned int i=0; i<actions.size(); ++i) { // select 0 if first case was not encountered
                    if(is_equal_to(actions[i].dsigma, 0.)) {
                        a = actions[i];
                        return i;
                    }
                }
            } else { // sigma < 0; select -dsigma or 0
                for(unsigned int i=0; i<actions.size(); ++i) { // select -dsigma
                    if(is_less_than(actions[i].dsigma, 0.)) {
                        a = actions[i];
                        return i;
                    }
                }
                for(unsigned int i=0; i<actions.size(); ++i) { // select 0 if first case was not encountered
                    if(is_equal_to(actions[i].dsigma, 0.)) {
                        a = actions[i];
                        return i;
                    }
                }
            }
        } else { // no lift
            if (!is_less_than(sig, angle_rate_magnitude)) { // sigma >= angle_rate_magnitude
                a.dsigma = -angle_rate_magnitude;
            } else if(!is_greater_than(sig, -angle_rate_magnitude)) { //sigma <= -angle_rate_magnitude
                a.dsigma = +angle_rate_magnitude;
            } else { // -angle_rate_magnitude < sigma < angle_rate_magnitude
                a.dsigma = 0.;
            }

            if(is_less_than(-angle_rate_magnitude, sig) && is_less_than(sig, angle_rate_magnitude)) { // -angle_rate_magnitude < sigma < angle_rate_magnitude; select dsig=0
                for(unsigned int i=0; i<actions.size(); ++i) {
                    if(is_equal_to(actions[i].dsigma, 0.)) {
                        a = actions[i];
                        return i;
                    }
                }
            } else if(!is_less_than(sig, angle_rate_magnitude)) { // sigma >= angle_rate_magnitude; select -dsigma
                for(unsigned int i=0; i<actions.size(); ++i) { // select -dsigma
                    if(is_less_than(actions[i].dsigma, 0.)) {
                        a = actions[i];
                        return i;
                    }
                }
            } else { // sigma <= -angle_rate_magnitude; select +dsigma
                for(unsigned int i=0; i<actions.size(); ++i) { // select +dsigma
                    if(is_greater_than(actions[i].dsigma, 0.)) {
                        a = actions[i];
                        return i;
                    }
                }
            }
        }
        return 0;
    }

    /**
     * @brief Select an action randomly
     * @param {const std::vector<beeler_glider_command> &} actions; available actions
     * @param {beeler_glider_command &} a; computed action
     * @return Indice of the chosen action
     */
    unsigned int random_policy(
        const std::vector<beeler_glider_command> &actions,
        beeler_glider_command &a)
    {
        a = actions[0]; //actions already randomized
        return 0;
    }

    /**
     * @brief Run the default policy and get the value of a rollout until the horizon
     * @param {const uct_node &} v; starting node
     * @param {unsigned int &} indice; first action's indice
     * @param {double &} delta; computed value
     */
    void default_policy(const uct_node &v, unsigned int &indice, double &delta) {
        delta = 0.;
        beeler_glider_state s_tp, s_t = v.s;
        beeler_glider_command a_t;
        switch(default_policy_selector) {
        case 0: { // random policy
            indice = random_policy(v.actions,a_t);
            break;
        }
        case 1: { // heuristic policy
            indice = heuristic_policy(v.actions,s_t,a_t);
            break;
        }
        }
        for(unsigned int t=0; t<horizon; ++t) {
            s_tp = transition_model(s_t,a_t);
            delta += pow(df,(double)t) * reward_model(s_t,a_t,s_tp);
            if(s_tp.is_out_of_bounds()){break;}
            s_t = s_tp;
            switch(default_policy_selector) {
            case 0: { // random policy
                random_policy(get_actions(s_t),a_t);
                break;
            }
            case 1: { // heuristic policy
                heuristic_policy(get_actions(s_t),s_t,a_t);
                break;
            }
            }
        }
    }

    /**
     * @brief Backup the value computed via the default policy to the parents & update the number of visit counter
     * @param {uct_node &} v; node
     * @param {unsigned int &} indice; indice of the Q value to update
     * @param {double &} delta; backed-up value
     * @note Recursive method
     */
    void backup(uct_node &v, unsigned int &indice, double &delta) {
        v.total_nb_visits += 1;
        v.nb_visits[indice] += 1;
        v.Q_values[indice] += 1./((double)v.nb_visits[indice]) * (delta - v.Q_values[indice]);
        if(v.depth > 0) {
            delta = v.parent->rewards[v.incoming_action_indice] + df * delta;
            backup(*v.parent, v.incoming_action_indice, delta);
        }
    }

    /**
     * @brief Get the greedy action wrt Q
     * @param {uct_node &} v; parent node
     * @param {beeler_glider_command &} a; resulting action
     */
    void greedy_action(uct_node &v, beeler_glider_command &a) {
        std::vector<double> scores;
        for(unsigned int i=0; i<v.children.size(); ++i) {
            scores.push_back(v.Q_values[i]);
        }
        a = v.actions[argmax(scores)];
    }

    /** @brief Roughly print the tree recursively as a standard output starting from an input node */
    void print_tree(uct_node &v) {
        v.print();
        for(auto &elt : v.children) {
            print_tree(elt);
        }
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
        uct_node v0(s0,nullptr,get_actions(s0),0,0); // root node
        for(unsigned int i=0; i<budget; ++i) {
            uct_node &v = tree_policy(v0);
            double delta = 0.;
            unsigned int indice = 0;
            default_policy(v,indice,delta);
            backup(v,indice,delta);
        }
        greedy_action(v0,a);
        a.dalpha = alpha_d_ctrl(s0); // D-controller
        print_tree(v0);//TRM
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
