#ifndef L2FSIM_UCT_NODE_HPP_
#define L2FSIM_UCT_NODE_HPP_

#include <cstdio>
#include <cstdlib>

/**
 * @file uct_node.hpp
 * @brief Node for MCTS for 'beeler_glider.hpp' model
 * @version 1.0
 * @since 1.0
 * @note compatibility: 'beeler_glider.hpp'; 'beeler_glider_state.hpp'; 'beeler_glider_command.hpp'
 */

namespace L2Fsim{

class uct_node {
public:
    /**
     * @brief Attributes
     * @param {beeler_glider_state} s; state of the node
     * @param {beeler_glider_command} incoming_action; action leading the node's parent to the node itself
     * @param {double} cumulative_reward; sum of the backed-up rewards
     * @param {unsigned int} number_of_visits;
     * @param {unsigned int} depth;
     * @param {uct_node *} parent; pointer to the parent node
     * @param {std::vector<beeler_glider_command>} avail_actions; actions available from the node's state
     * @param {std::vector<uct_node>} children; vector containing the children of the node, initialy empty
     *
     * @warning the pointer to the parent 'parent' is obsolete if the node is root, make use of the boolean 'is_root_node'
     */
    beeler_glider_state s;
    beeler_glider_command incoming_action;
    double cumulative_reward;
    unsigned int number_of_visits;
    unsigned int depth;
    uct_node *parent;
    std::vector<beeler_glider_command> avail_actions;
    std::vector<uct_node> children;

    /** @brief Empty constructor */
    uct_node() {}

    /** @brief Constructor with given state */
    uct_node(
        beeler_glider_state _s,
        std::vector<beeler_glider_command> _avail_actions,
        double _cumulative_reward=0.,
        unsigned int _number_of_visits=0,
        unsigned int _depth=0) :
        s(_s),
        cumulative_reward(_cumulative_reward),
        number_of_visits(_number_of_visits),
        depth(_depth),
        avail_actions(_avail_actions)
    {}

    /** @brief Constructor without state */
    uct_node(
        std::vector<beeler_glider_command> _avail_actions,
        double _cumulative_reward=0.,
        unsigned int _number_of_visits=0,
        unsigned int _depth=0) :
        s(),
        cumulative_reward(_cumulative_reward),
        number_of_visits(_number_of_visits),
        depth(_depth),
        avail_actions(_avail_actions)
    {}

    /** @brief Return a reference on the state attribute */
    beeler_glider_state & get_state() {return s;}

    /** @brief Boolean test for a node being fully expanded or not */
    bool is_fully_expanded() {
        return (avail_actions.size() == 0) ? true : false;
    }

    /** @brief Get the average reward */
    double get_average_reward() {
        return cumulative_reward / (double)number_of_visits;
    }

    /**
     * @brief Get a reference on the last child that was created i.e. end position of 'children' attribute
     * @return {uct_node &} reference on the last child
     */
    uct_node & get_last_child() {
        return children.back();
    }
};

}

#endif
