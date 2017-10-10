#ifndef L2FSIM_UCT_NODE_HPP_
#define L2FSIM_UCT_NODE_HPP_

#include <cstdio>
#include <cstdlib>

/**
 * @file uct_node.hpp
 * @brief Node for MCTS for 'beeler_glider.hpp' model
 * @version 1.1
 * @since 1.0
 * @note compatibility: 'beeler_glider.hpp'; 'beeler_glider_state.hpp'; 'beeler_glider_command.hpp'
 */

namespace L2Fsim{

class uct_node {
public:
    beeler_glider_state s; ///< State of the node
    uct_node *parent; ///< Pointer to the parent node
    std::vector<beeler_glider_command> actions; ///< Available actions
    std::vector<double> values; ///< state-action values
    std::vector<double> rewards; ///< state-action rewards
    std::vector<unsigned> nb_visits; ///< state-action number of visits
    std::vector<uct_node> children; ///< state-action resulting children
    unsigned incoming_action_indice; ///< indice of the action taken from the parent node
    unsigned depth; ///< Depth of the node
    unsigned total_nb_visits; ///< Total number of visits

    /** @brief Empty constructor */
    uct_node() {}

    /** @brief Constructor */
    uct_node(
        beeler_glider_state _s,
        uct_node *_parent,
        std::vector<beeler_glider_command> _actions,
        unsigned _incoming_action_indice,
        unsigned _depth = 0) :
        s(_s),
        parent(_parent),
        actions(_actions),
        incoming_action_indice(_incoming_action_indice),
        depth(_depth)
    {
        unsigned sz = actions.size();
        values = std::vector<double>(sz,0.);
        rewards = std::vector<double>(sz,0.);
        nb_visits = std::vector<unsigned>(sz,0);
        total_nb_visits = 0;
    }

    /**
     * @brief Next expansion action
     *
     * Get the next expansion action among the available actions.
     */
    beeler_glider_command get_next_expansion_action() const {
        return actions.at(children.size());
    }

    int indic(const double &a) {
        if(is_less_than(a,0.)) {
            return -1;
        } else if(is_equal_to(a,0.)) {
            return 0;
        } else {
            return 1;
        }
    }

    /**
     * @brief Print
     *
     * Print some attributes of the node as a standard output.
     */
    void print() {
        std::string sep = " ";
        std::cout << "d" << depth << sep;
        std::cout << "nv:" << total_nb_visits << sep;

        for(unsigned i=0; i<children.size(); ++i) {
            std::cout << nb_visits.at(i) << "|";
        }
        std::cout << sep;

        std::cout << "Q:";
        for(unsigned i=0; i<children.size(); ++i) {
            std::cout << values.at(i) << "|";
        }
        std::cout << sep;

        if(depth>0) {
            std::cout << "ia:" << indic(parent->actions.at(incoming_action_indice).dsigma) << sep;
        }

        std::cout << "a:";
        for (auto &ac : actions) {
            std::cout << indic(ac.dsigma) << "|";
        }
        std::cout << sep;

        std::cout << "chld:" << children.size() << sep;
        std::cout << "sg:" << s.sigma << sep;
        std::cout << "z:" << s.z << sep;
        std::cout << "E:" << s.z + s.V * s.V / (2. * 9.81) << sep;
        std::cout << "Ed:" << s.zdot + s.V * s.Vdot / 9.81 << "\n";
        //std::cout << "this = " << this << " ";
        //std::cout << "parent = " << parent << "\n";
    }

    /**
     * @brief Is terminal
     *
     * Boolean test for termination criterion.
     * @return Return true if the node is terminal.
     */
    bool is_terminal() {
        return s.is_out_of_bounds();
    }

    /**
     * @brief Is fully expanded
     *
     * Boolean test for a node to be fully expanded or not. A node is fully expanded if all
     * of its available actions have been tried at least once and consequently the number of
     * children is equal to the number of actions.
     * @return Return true if the node is fully expanded.
     */
    bool is_fully_expanded() {
        return (children.size() == actions.size()) ? true : false;
    }
};

}

#endif
