#ifndef L2FSIM_B03_UCT_PILOT_HPP_
#define L2FSIM_B03_UCT_PILOT_HPP_

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <L2Fsim/pilot/pilot.hpp>
#include <L2Fsim/pilot/mcts/b03_node.hpp>
#include <L2Fsim/flight_zone/flat_thermal_soaring_zone.hpp>

/**
 * @file b03_uct_pilot.hpp
 * @brief An online-anytime implementation of a UCT algorithm
 * @version 1.0
 * @since 1.0
 *
 * @note compatibility: 'flat_thermal_soaring_zone.hpp'; 'beeler_glider.hpp'; 'beeler_glider_state.hpp'; 'beeler_glider_command.hpp'
 * @note make use of: 'b03_node.hpp'
 * @note the different actions available from a node's state are set via the method 'get_expendable_actions'
 * @note transition model is defined in function 'transition_model'
 * @note reward model is defined in function 'reward_model'
 * @note termination criterion for a node is set in 'is_terminal' method
 */

namespace L2Fsim{

class b03_uct_pilot : public pilot {
public:
    /**
     * Attributes
     * @param {beeler_glider} ac; aircraft model
     * @param {flat_thermal_soaring_zone} fz; atmosphere model
     * @param {void (*transition_function)(aircraft &, flight_zone &, double &, const double &, const double &)}
     * @param {double} angle_rate_magnitude; magnitude of the increment that one can apply to the angles
     * @param {double} uct_parameter; parameter for the UCT formula
     * @param {double} time_step_width;
     * @param {double} sub_time_step_width;
     * @param {double} df; discount factor
     * @param {unsigned int} horizon; time limit for online simulations
     * @param {unsigned int} computational_budget; number of expanded nodes in the tree
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
    unsigned int computational_budget;

    b03_uct_pilot(
        void (*_transition_function)(aircraft &, flight_zone &, double &, const double &, const double &),
        beeler_glider _ac,
        flat_thermal_soaring_zone _fz,
        double _angle_rate_magnitude=.01,
        double _uct_parameter=1.,
        double _time_step_width=1e-1,
        double _sub_time_step_width=1e-1,
        double _df=.9,
        unsigned int _horizon=100,
        unsigned int _computational_budget=1000) :
        ac(_ac),
        fz(_fz),
        transition_function(_transition_function),
        angle_rate_magnitude(_angle_rate_magnitude),
        uct_parameter(_uct_parameter),
        time_step_width(_time_step_width),
        sub_time_step_width(_sub_time_step_width),
        df(_df),
        horizon(_horizon),
        computational_budget(_computational_budget)
    {}

    /**
     * @brief Boolean test for termination criterion
     * @param {const beeler_glider_state &} _s; tested state
     */
    bool is_terminal(const beeler_glider_state &_s) {
        bool answer = (_s.z < 0.) ? true : false;
        return answer;
    }

    /**
     * @brief Compute the UCT score of a node
     * @param {b03_node &} v; considered node
     * @return {double} score
     */
    double uct_score(b03_node &v) {
        double nvis = (double) v.number_of_visits;
        assert(nvis != 0.);
        double nparent = (double) v.parent->number_of_visits;
        double avg_rwd = v.get_average_reward();
        return avg_rwd + 2. * uct_parameter * sqrt(2. * log(nparent) / nvis);
    }

    /**
     * @brief Get a reference on the 'best' child according to the UCT criteria
     * @param {const b03_node &} parent; parent node
     * @return {b03_node} best UCT child
     */
    b03_node & best_uct_child(b03_node &parent) {
        std::vector<double> scores;
        std::vector<unsigned int> max_indices;
        for(unsigned int i=0; i<parent.children.size(); ++i) {
            scores.push_back(uct_score(parent.children[i]));
        }
        sort_indices(scores,max_indices);
        //std::cout<<"best uct child ("<<parent.children.size()<<" children)";//TRM
        //std::cout<<" scores: "<<scores[0]<<" "<<scores[1]<<" "<<scores[2];//TRM
        //std::cout<<" - hence max_indices has size "<<max_indices.size()<<std::endl;//TRM
        return parent.children.at(rand_element(max_indices));
    }

    /**
     * @brief Get the available actions for a node
     * @return {std::vector<beeler_glider_command>} vector of the available actions
     */
    std::vector<beeler_glider_command> get_expendable_actions() {
        std::vector<beeler_glider_command> v;
        v.push_back(beeler_glider_command(0.,0.,+angle_rate_magnitude));
        v.push_back(beeler_glider_command(0.,0.,0.));
        v.push_back(beeler_glider_command(0.,0.,-angle_rate_magnitude));
        return v;
    }

    /**
     * @brief Transition function model
     * @param {const beeler_glider_state &} s; current state
     * @param {const beeler_glider_command &} a; applied command
     * @return {beeler_glider_state} resulting state
     * @warning dynamic cast
     */
    beeler_glider_state transition_model(const beeler_glider_state &s, const beeler_glider_command &a) {
        beeler_glider_state s_prime = s;
        ac.set_state(s_prime);
        ac.set_command(a);
        double current_time = s_prime.time;
        transition_function(ac,fz,current_time,time_step_width,sub_time_step_width);
        s_prime = dynamic_cast <beeler_glider_state &> (ac.get_state()); // retrieve the computed state
        return s_prime;
    }

    /**
     * @brief Reward function model
     * @param {const beeler_glider_state &} s_t; current state
     * @param {const beeler_glider_command &} a_t; applied command
     * @param {const beeler_glider_state &} s_tp; resulting state
     * @return {double} computed reward
     */
    double reward_model(
        const beeler_glider_state &s_t,
        const beeler_glider_command &a_t,
        const beeler_glider_state &s_tp)
    {
        (void)a_t; (void)s_tp; // unused by default
        double edot = s_t.zdot + s_t.V * s_t.Vdot / 9.81;
        double rwd=0.;
        if(edot>0.){rwd=1.;}
        return rwd;
    }

    /**
     * @brief Create a new child corresponding to an untried action
     * @param {b03_node &} v; parent node
     * @note link the child to the current node as a parent
     * @note remove the selected action from 'expendable_actions' attribute
     * @return {void}
     */
    void create_child(b03_node &v) {
        int indice = rand_indice(v.expendable_actions);
        beeler_glider_command a = v.expendable_actions.at(indice);
        v.expendable_actions.erase(v.expendable_actions.begin()+indice);
        beeler_glider_state s_prime = transition_model(v.s,a);
        b03_node new_child(s_prime,get_expendable_actions(),0.,1,v.depth+1);
        new_child.incoming_action = a;
        new_child.parent = &v;
        v.children.push_back(new_child);
    }

    /**
     * @fn
     * @brief Apply the tree policy from 'root' node to leaf node, there are 3 cases:
     * 1. The root node is terminal: return the root node;
     * 2. The root node is fully expanded: get the 'best' child according to UCT criteria and recursively run the function on this child;
     * 3. The root node is not fully expanded: return a new child
     * @param {b03_node &} v0; parent node
     * @param {b03_node &} v; child node
     * @note recursive function
     */
    void tree_policy(b03_node &v0, b03_node &v) {
        //std::cout<<"    - tree plc with "<<v0.children.size()<<" children and ";//TRM
        //std::cout<<v0.expendable_actions.size()<<" expendable actions";
        if(is_terminal(v0.s)) {
            v = v0;
        } else {
            if(v0.is_fully_expanded()) {
                assert(v0.expendable_actions.size() == 0); //TODO: remove
                tree_policy(best_uct_child(v0),v);
            } else {
                create_child(v0);
                v = v0.get_last_child();
            }
        }
    }

    /**
     * @brief Run the default policy and compute the reward
     * @param {const beeler_glider_state &} s; starting state
     * @param {double &} reward; computed reward
     */
    void default_policy(const beeler_glider_state &s, double &reward) {
        std::vector<beeler_glider_command> actions = get_expendable_actions();
        beeler_glider_state s_tp, s_t=s;
        beeler_glider_command a_t;
        for(unsigned int t=0; t<horizon; ++t) {
            //std::cout<<"dft policy"<<std::endl;//TRM
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
    void backup(b03_node &v, double &reward) {
        v.number_of_visits += 1;
        v.cumulative_reward += reward;
        reward *= df; // apply discount for parent
        if(v.depth > 0) {backup(*v.parent,reward);}
    }

    /**
     * @brief Get the action leading to the child with the highest average reward
     * @param {b03_node &} v0; parent node
     * @param {beeler_glider_command &} a; computed action
     */
    void best_action(b03_node &v0, beeler_glider_command &a) {
        std::vector<double> scores;
        std::vector<unsigned int> max_indices;
        for(unsigned int i=0; i<v0.children.size(); ++i) {
            scores.push_back(v0.children[i].get_average_reward());
        }
        sort_indices(scores,max_indices);
        //std::cout<<"best actions"<<std::endl;//TRM
        b03_node v = v0.children.at(rand_element(max_indices));
        a = v.incoming_action;
    }

    /**
     * @brief Tree computation and action selection
     * @param {state &} _s; reference on the state
     * @param {command &} _a; reference on the command
     * @warning dynamic cast of state and action
     */
	pilot & operator()(state &_s, command &_a) override
	{
        beeler_glider_state &s0 = dynamic_cast <beeler_glider_state &> (_s);
        beeler_glider_command &a = dynamic_cast <beeler_glider_command &> (_a);
        b03_node v0(s0,get_expendable_actions(),0.,1,0); // root node
        for(unsigned int i=0; i<computational_budget; ++i) {
            b03_node v(get_expendable_actions(),0.,0,0);
            double reward=0.;
            tree_policy(v0,v);
            default_policy(v.get_state(),reward);
            backup(v,reward);
        }
        best_action(v0,a);

        // D-controller
        double kd=1e-2, gammadot_ref=0.;
        a.dalpha = kd * (gammadot_ref - s0.gammadot);

        //// TO REMOVE //////////////////////////////////////////////////////////////////
        if(true){//TRM
        for(unsigned int l=0; l<v0.children.size(); ++l) {
            std::cout<<"visit "<<v0.children[l].number_of_visits;
            std::cout<<" avgreward "<<v0.children[l].get_average_reward();
            std::cout<<" uctscore "<<uct_score(v0.children[l])<<std::endl;
        }
        }

        if(false){//TRM
        std::vector<double> wtest;
        std::cout<<"stest: ";
        fz.wind(0.,0.,500.,200.,wtest);
        for(unsigned int i=0; i<wtest.size(); ++i) {
            std::cout<<wtest[i]<<" ";
        }
        std::cout<<std::endl;
        }
        /////////////////////////////////////////////////////////////////////////////////
		return *this;
	}

    /**
     * @brief Policy for 'out of range' situations
     * @param {state &} s; reference on the state
     * @param {command &} a; reference on the command
     */
    pilot & out_of_range(state &_s, command &_a) override
    {
        beeler_glider_state &s = dynamic_cast <beeler_glider_state &> (_s);
        beeler_glider_command &a = dynamic_cast <beeler_glider_command &> (_a);
        a.dalpha = 0.;
        a.dbeta = 0.;
        if(s.sigma < 0.4) {
            a.dsigma = +angle_rate_magnitude;
        } else {
            a.dsigma = 0.;
        }
		return *this;
    }
};

}

#endif
