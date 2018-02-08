#ifndef L2FSIM_OPTIMISTIC_PILOT_HPP_
#define L2FSIM_OPTIMISTIC_PILOT_HPP_

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <map>
#include <pilot.hpp>
#include <optimistic/optimistic_node.hpp>
#include <flat_thermal_soaring_zone.hpp>

/**
 * @file optimistic_pilot.hpp
 * @brief An online-anytime implementation of an optimistic planning algorithm (OPD)
 * @version 1.0 (based on uct_pilot code)
 * @since 1.0
 * @note compatibility: 'flat_thermal_soaring_zone.hpp'; 'beeler_glider.hpp'; 'beeler_glider_state.hpp'; 'beeler_glider_command.hpp'
 * @note make use of: 'optimistic_node.hpp'
 * @note the different actions available from a node's state are set via the method 'get_actions'
 * @note transition model is defined in function 'transition_model'
 * @note reward model is defined in function 'reward_model'
 * @note termination criterion for a node is set in 'is_terminal' method
 */

namespace L2Fsim{

class optimistic_pilot : public pilot {
public:
    /**
     * @brief Attributes
     * @param {beeler_glider} ac; aircraft model
     * @param {flat_thermal_soaring_zone} fz; atmosphere model
     * @param {double} angle_rate_magnitude; magnitude of the increment that one can apply to the angles
     * @param {double} kdalpha; coefficient for the D controller in alpha
     * @param {double} time_step_width;
     * @param {double} sub_time_step_width;
     * @param {double} df; discount factor
     * @param {unsigned int} budget; number of expanded nodes in the tree
     * @param {std::multimap<double, optimistic_node*>} leaves; map of the leaves, ordered by b_value, initially empty
     * @param {optimistic_node *} u_max_node; pointer to the node with u_value maximum u_max
     */
    beeler_glider ac;
    flat_thermal_soaring_zone fz;
    double angle_rate_magnitude;
    double kdalpha;
    double time_step_width;
    double sub_time_step_width;
    double df;
    unsigned int budget;
    std::multimap<double, optimistic_node*> leaves;
    optimistic_node *u_max_node;

    /** @brief Constructor */
    optimistic_pilot(
        beeler_glider &_ac,
        std::string sc_path,
        std::string envt_cfg_path,
        double noise_stddev,
        double _angle_rate_magnitude=.01,
        double _kdalpha=.01,
        double _time_step_width=1e-1,
        double _sub_time_step_width=1e-1,
        double _df=.9,
        unsigned int _budget=10000) :
        ac(_ac),
        fz(sc_path,envt_cfg_path,noise_stddev),
        angle_rate_magnitude(_angle_rate_magnitude),
        kdalpha(_kdalpha),
        time_step_width(_time_step_width),
        sub_time_step_width(_sub_time_step_width),
        df(_df),
        budget(_budget)
	{}

    /**
     * @brief Reward function model
     * @param {const beeler_glider_state &} s_t; current state
     * @return {double} computed instantaneous reward
     */
    double reward_model(const beeler_glider_state &s_t) {
        double edot = s_t.zdot + s_t.V * s_t.Vdot / 9.81;
        return sigmoid(edot,10.,0.);
    }

    /**
     * @brief Compute the u_value & b_value of a node
     * @param {optimistic_node &} v; considered node
     */
    void compute_values(optimistic_node &v) {
        double df_d = pow(df, v.depth-1);
        v.u_value = v.parent->u_value + df_d * v.parent->reward;
        v.b_value = v.u_value + df_d*df/ (1.-df);
    }

    /**
     * @brief Set the value of dalpha with a D-controller in order to soften the phugoid behaviour
     * @param {beeler_glider_state &} s; state
     * @param {beeler_glider_command &} a; modified action
     */
    void alpha_d_ctrl(const beeler_glider_state &s, beeler_glider_command &a) {
        a.dalpha = kdalpha * (0. - s.gammadot);
    }

    /**
     * @brief Get the available actions for a node, given its state
     * @param {const beeler_glider_state &} s; state of the node
     * @return {std::vector<beeler_glider_command>} vector of the available actions
     */
    std::vector<beeler_glider_command> get_actions(const beeler_glider_state &s) {
        std::vector<beeler_glider_command> vect_a;
        double sig = s.sigma;
        double mam = s.max_angle_magnitude;
        if(sig+angle_rate_magnitude < +mam) {
            vect_a.push_back(beeler_glider_command(0.,0.,+angle_rate_magnitude));
        }
        if(sig-angle_rate_magnitude > -mam) {
            vect_a.push_back(beeler_glider_command(0.,0.,-angle_rate_magnitude));
        }
        vect_a.push_back(beeler_glider_command(0.,0.,0.));
        for (auto &action : vect_a) {
            alpha_d_ctrl(s,action);
        }
        assert(vect_a.size()!=0);
        return vect_a;
    }

    /** @brief Print informations about the set of leaves */
	void print_leaves(){
		for(auto &e : leaves){
			std::cout << std::get<1>(e)->avail_actions.size() << "-";
			std::cout << std::get<1>(e)->depth << "-";
			std::cout << std::get<1>(e)->u_value << "  -  ";
		}
		std::cout << std::endl;
    }

    /**
     * @brief Create all children from a node
     * @param {optimistic_node *} ptr; pointer to the parent node
     * @param {beeler_glider_command &} a; applied command
     * @note Link the child to the current node as a parent
     * @note Emplace the created child in the map of leaves
     * @return {void}
     */
    void create_child(optimistic_node *ptr, beeler_glider_command &a) {
        beeler_glider_state s_p = transition_model(ptr->s,a);
        double rwd = reward_model(s_p);
        unsigned int new_depth = ptr->depth + 1;
        ptr->children.emplace_back(s_p, get_actions(s_p), a, rwd,0.,0.,new_depth, ptr);
        compute_values(ptr->children.back());
        leaves.emplace(ptr->children.back().b_value,&ptr->children.back());
        if(!is_less_than(ptr->children.back().u_value, u_max_node->u_value)){
            u_max_node = &ptr->children.back();
        }
	}

    /**
     * @brief Expand the node v with highest b_value
     * @param {optimistic_node *} ptr; pointer to the node to expand
     * @return {void}
     */
    void expand(optimistic_node *ptr) {
        leaves.erase(--leaves.end());
        for(auto &a : ptr->avail_actions) {
          	create_child(ptr, a);
        }
    }

    /**
     * @brief Get the best action starting from the root, corresponding to the leaf with the highest u_value
     * @return {beeler_glider_command} the best action
     */
    beeler_glider_command get_best_action() {
        beeler_glider_command best_a;
        optimistic_node *v = u_max_node;
     	while(v->depth != 0) {
            best_a = v->incoming_action;
            v = v->parent;
        }
        return best_a;
    }

    /**
     * @brief Transition function; perform a transition given: an aircraft model with a correct state and command; an atmospheric model; the current time; the time-step-width and the sub-time-step-width
     * @note static method for use within an external simulator
     * @param {aircraft &} ac; aircraft model
     * @param {flight_zone &} fz; atmosphere model
     * @param {double &} current_time; current time
     * @param {const double} time_step_width; time-step-width
     * @param {const double} sdt; sub-time-step-width
     */
    static void transition_function(
        aircraft &ac,
        flight_zone &fz,
        double &current_time,
        const double time_step_width,
        const double sdt)
    {
        for(unsigned int n=0; n<(unsigned int)(time_step_width/sdt); ++n) {
            ac.apply_command();
            ac.update_state_dynamic(fz,current_time,ac.get_state());
            ac.get_state().apply_dynamic(sdt);
            current_time += sdt;
            ac.get_state().set_time(current_time);
        }
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
        double current_time = s_p.time;
        transition_function(ac,fz,current_time,time_step_width,sub_time_step_width);
        s_p = dynamic_cast <beeler_glider_state &> (ac.get_state()); // retrieve the computed state
        return s_p;
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
        double rew_0 = reward_model(s0);
        optimistic_node v0(s0,get_actions(s0),rew_0,0.,0.,0); // root node
        leaves.insert(std::pair<double,optimistic_node*> (v0.b_value,&v0));
        u_max_node = &v0;
        for(unsigned int i=0; i<budget; ++i) {
           	expand((--leaves.end())->second);
        }
        a = get_best_action();
        alpha_d_ctrl(s0,a); // D-controller
        //std::cout<<"ACTION choosen :  dsigma = " << a.dsigma << std::endl;
        //std::cout<<"                Altitude = " << s0.z     << std::endl;
        leaves.clear();
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
