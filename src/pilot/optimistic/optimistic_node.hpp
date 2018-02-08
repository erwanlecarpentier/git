#ifndef L2FSIM_OPTIMISTIC_NODE_HPP_
#define L2FSIM_OPTIMISTIC_NODE_HPP_

#include <cstdio>
#include <cstdlib>

/**
 * @file optimistic_node.hpp
 * @brief Node for optimistic planning for 'beeler_glider.hpp' model
 * @version 1.0 (based on uct_node code)
 * @since 1.0
 * @note compatibility: 'beeler_glider.hpp'; 'beeler_glider_state.hpp'; 'beeler_glider_command.hpp'
 */

namespace L2Fsim{

class optimistic_node {
public:
    /**
     * @brief Attributes
     * @param {beeler_glider_state} s; state of the node
     * @param {std::vector<beeler_glider_command>} avail_actions; actions available from the node's state
     * @param {beeler_glider_command} incoming_action; action leading the node's parent to the node itself
     * @param {double} reward; reward obtained for this state s following this action incoming_action
     * @param {double} u_value; u_value of the node
     * @param {double} b_value; b_value of the node
     * @param {unsigned int} depth; depth of the node in the tree, root depth is 0
     * @param {optimistic_node *} parent; pointer to the parent node
     * @param {std::vector<optimistic_node>} children; vector containing the children of the node, initialy empty
     * @warning the pointer to the parent 'parent' is obsolete if the node is root, make use of the boolean 'is_root_node'
     */
    beeler_glider_state s;
    std::vector<beeler_glider_command> avail_actions;
    beeler_glider_command incoming_action;
    double reward;
    double u_value;
    double b_value;
    unsigned int depth;
    optimistic_node* parent;
    std::vector<optimistic_node> children;

    /** @brief Empty constructor */
    optimistic_node() {
		children.reserve(3);
		avail_actions.reserve(3);
	}

    /** @brief Constructor with given state */
    optimistic_node(
        beeler_glider_state _s,
        std::vector<beeler_glider_command> _avail_actions,
        beeler_glider_command _incoming_action,
        double _reward=0.,
        double _u_value=0.,
        double _b_value=0.,
        unsigned int _depth=0,
        optimistic_node* _parent=nullptr) :
        s(_s),
        avail_actions(_avail_actions),
        incoming_action(_incoming_action),
        reward(_reward),
        u_value(_u_value),
        b_value(_b_value),
        depth(_depth),
        parent(_parent)
    {
		children.reserve(3);
		avail_actions.reserve(3);
	}
};

}

#endif
