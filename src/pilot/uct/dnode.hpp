#ifndef DNODE_HPP_
#define DNODE_HPP_

#include<uct/cnode.hpp>

/**
 * @brief Decision node class
 */
template <class ST, class AC>
class dnode {
public:
    typedef ST ST_type; ///< State type
    typedef AC AC_type; ///< Action type

    ST s; ///< Labelling state
    std::vector<AC> actions; ///< Available actions, iteratively removed
    cnode * parent; ///< Pointer to parent node
    std::vector<std::unique_ptr<cnode>> children; ///< Child nodes
    //unsigned nvis; ///< Number of visits//TRM?

    /**
     * @brief Constructor
     */
    dnode(
        ST _s,
        std::vector<AC> _actions,
        cnode * _parent) :
        s(_s),
        actions(_actions),
        parent(_parent)
    {
        //nvis = 0;//TRM?
    }

    /**
     * @brief Create Child
     *
     * Create a child (hence a chance node).
     * The action of the child is randomly selected.
     * @return Return the sampled action.
     * @warning Remove the sampled action from the actions vector.
     */
    AC create_child() {
        unsigned indice = rand_indice(actions);
        AC sampled_action = actions.at(indice);
        actions.erase(actions.begin() + indice);
        children.emplace_back(
            std::unique_ptr<cnode>(new cnode(s,sampled_action))
        );
        return sampled_action;
    }

    /**
     * @brief Get children values
     *
     * Get an ordered vector containing all the values of the children.
     * * @return Return the vector containing all the children values.
     */
    std::vector<double> get_children_values() const {
        std::vector<double> children_values;
        for(auto &c : children) {
            children_values.emplace_back(c->get_value());
        }
        return children_values;
    }

    /**
     * @brief Get value
     *
     * Get the value of the node.
     * This is the maximum value of its children.
     * @return Return the value of the node.
     */
    double get_value() const {
        std::vector<double> children_values = get_children_values();
        return children.at(argmax(children_values))->get_value();
    }

    /**
     * @brief Is fully expanded
     *
     * Test whether the node is fully expanded ie if every actions have been sampled.
     * Node is a leaf.
     * @return Return a boolean answer to the test.
     */
    bool is_fully_expanded() const {
        return (actions.size() == 0);
    }
};

#endif // DNODE_HPP_
