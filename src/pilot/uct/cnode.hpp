#ifndef CNODE_HPP_
#define CNODE_HPP_

class dnode; // forward declaration

/**
 * @brief Chance node class
 */
template <class ST, class AC>
class cnode {
public:
    typedef ST ST_type; ///< State type
    typedef AC AC_type; ///< Action type

    ST s; ///< Labelling state
    AC a; ///< Labelling action
    std::vector<std::unique_ptr<dnode>> children; ///< Child nodes
    std::vector<double> sampled_returns; ///< Sampled returns
    //unsigned nvis; ///< Number of visits //TRM?

    /**
     * @brief Constructor
     */
    cnode(
        ST _s,
        AC _a) :
        s(_s),
        a(_a)
    {
        //nvis = 0;//TRM?
    }

    dnode * get_last_child() const {
        return children.back().get();
    }

    /**
     * @brief Get value
     *
     * Get the value of the node.
     * This is the mean of the sampled returns.
     * @return Return the value of the node.
     */
    double get_value() const {
        return (1. / (double) sampled_returns.size()) * std::accumulate(sampled_returns.begin(),sampled_returns.end(),0.0);
    }
};

#endif // CNODE_HPP_
