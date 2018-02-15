#ifndef UCT_HPP_
#define UCT_HPP_

#include<cassert>
#include<vector>
#include<memory>
#include<numeric>

#include<uct/cnode.hpp>
#include<uct/dnode.hpp>
#include<utils.hpp>

/**
 * @brief UCT algorithm class
 */
template <class ST, class AC class MD, class PL>
class uct {
public:
    typedef ST ST_type; ///< State type
    typedef AC AC_type; ///< Action type
    typedef MD MD_type; ///< Model type
    typedef PL PL_type; ///< Policy type (default policy)

    MD model; ///< Generative model
    PL default_policy; ///< Default policy
    double discount_factor; ///< Discount factor
    double uct_parameter; ///< UCT parameter
    unsigned budget; ///< Budget ie number of expanded nodes in the tree
    unsigned global_counter; ///< Global counter of number of visits / nodes expansions
    unsigned horizon; ///< Horizon for the default policy simulation

    /**
     * @brief Constructor
     */
    uct(const parameters &p, environment *en) :
        model(*en),
        default_policy(p,en)
    {
        // use the specific parameters of the given model
        model.misstep_probability = p.MODEL_MISSTEP_PROBABILITY;
        model.state_gaussian_stddev = p.MODEL_STATE_GAUSSIAN_STDDEV;
        global_counter = 0;
        uct_parameter = p.UCT_CST;
        budget = p.TREE_SEARCH_BUDGET;
        discount_factor = p.DISCOUNT_FACTOR;
        horizon = p.DEFAULT_POLICY_HORIZON;
    }

    /**
     * @brief Sample return
     *
     * Sample a return with the default policy starting at the input state.
     * @param {ST} s; input state
     * @return Return the sampled return.
     */
    double sample_return(ST s) {
        if(model.is_terminal(s)) {
            return 0.;
        }
        double total_return = 0.;
        AC a = default_policy(s);
        for(unsigned t=0; t<horizon; ++t) {
            ST s_p;
            model.state_transition(s,a,s_p);
            total_return += pow(discount_factor,(double)t) * model.reward_function(s,a,s_p);
            if(model.is_terminal(s_p)) {
                break;
            }
            s = s_p;
            a = default_policy(s);
        }
        return total_return;
    }

    /**
     * @brief Update value
     *
     * Update the value of a chance node by stacking a new sampled value.
     * @param {cnode<ST,AC> *} ptr; pointer to the updated chance node
     * @param {double} q; new sampled value
     */
    void update_value(cnode<ST,AC> * ptr, double q) const {
        ptr->sampled_returns.push_back(q);
    }

    /**
     * @brief Select child
     *
     * Select child of a decision node wrt the UCT tree policy.
     * The node must be fully expanded.
     * @param {dnode<ST,AC> *} v; decision node
     * @return Return to the select child, which is a chance node.
     */
    cnode<ST,AC> * select_child(dnode<ST,AC> * v) const {
        std::vector<double> children_uct_scores;
        for(auto &c : v->children) {
            children_uct_scores.emplace_back(
                c->get_value()
                + 2 * uct_parameter *
                sqrt(log((double) global_counter) / ((double) c->sampled_returns.size()))
            );
        }
        return v->children.at(argmax(children_uct_scores)).get();
    }

    /**
     * @brief Evaluate
     *
     * Create a new child node to a decision node and sample a return value with the default
     * policy.
     * @param {dnode<ST,AC> *} v; pointer to the decision node
     * @return Return the sampled value.
     */
    double evaluate(dnode<ST,AC> * v) {
        ST s_p;
        global_counter++; // a chance node will be created
        model.state_transition(v->s,v->create_child(),s_p);
        double q = sample_return(s_p);
        update_value(v->children.back().get(),q);//TOCHECK that we have to do this
        return q;
    }

    /**
     * @brief Is state already sampled
     *
     * Equality operator.
     * @param {cnode<ST,AC> *} ptr; pointer to the chance node
     * @param {ST &} s; sampled state
     * @param {unsigned &} ind; indice modified to the value of the indice of the existing
     * decision node with state s if the comparison succeeds.
     */
    bool is_state_already_sampled(cnode<ST,AC> * ptr, ST &s, unsigned &ind) const {
        for(unsigned i=0; i<ptr->children.size(); ++i) {
            if(s.is_equal_to(ptr->children[i]->s)) {
                ind = i;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Search tree
     *
     * Search within the tree, starting from the input decision node.
     * Recursive method.
     * @param {dnode<ST,AC> *} v; input decision node
     * @return Return the sampled return at the given decision node
     */
    double search_tree(dnode<ST,AC> * v) {
        if(model.is_terminal(v->s)) { // terminal node
            return 0.;
        } else if(!v->is_fully_expanded()) { // leaf node, expand it
            return evaluate(v);
        } else { // apply UCT tree policy
            cnode<ST,AC> * ptr = select_child(v);
            ST s_p;
            model.state_transition(v->s,ptr->a,s_p);
            double r = model.reward_function(v->s,ptr->a,s_p);
            double q = 0.;
            unsigned ind = 0; // indice of resulting child
            if(is_state_already_sampled(ptr,s_p,ind)) { // go to node
                q = r + discount_factor * search_tree(ptr->children.at(ind).get());
            } else { // leaf node, create a new node
                ptr->children.emplace_back(std::unique_ptr<dnode<ST,AC>>(new dnode<ST,AC>(s_p,model.get_action_space(s_p),ptr)));
                q = r + discount_factor * evaluate(ptr->get_last_child());
            }
            update_value(ptr,q);
            return q;
        }
        /* // Original implementation (slower)
        if(model.is_terminal(v->s)) { // terminal node
            return 0.;
        } else if(v->is_leaf()) { // leaf node, expand it
            return evaluate(v->s);
        } else { // apply UCT tree policy
            cnode<ST,AC><ST,AC> * ptr = select_child(v);
            ST s_p;
            model.state_transition(v->s,ptr->a,s_p);
            double r = model.reward_function(v->s,ptr->a,s_p);
            ptr->children.emplace_back(std::unique_ptr<dnode<ST,AC>>(new dnode<ST,AC>(s_p,model.get_action_space(s_p),ptr)));
            double q = r + discount_factor * search_tree(ptr->get_last_child());
            update_value(ptr,q);
            return q;
        }
        */
    }

    /**
     * @brief Build UCT tree
     *
     * Build a tree at the input root node.
     * @param {dnode<ST,AC> &} root; reference to the input root node
     */
    void build_uct_tree(dnode<ST,AC> &root) {
        for(unsigned i=0; i<budget; ++i) {
            search_tree(&root);
        }
        global_counter = 0;
    }

    /**
     * @brief Argmax value
     *
     * Get the indice of the child with the maximum value of an input decision node.
     * @param {const dnode<ST,AC> &} v; input decision node
     * @return Return the indice of the child with the maximum value
     */
    unsigned argmax_value(const dnode<ST,AC> &v) const {
        std::vector<double> values;
        for(auto &ch: v.children) {
            values.emplace_back(ch->get_value());
        }
        return argmax(values);
    }

    /**
     * @brief Recommended action
     *
     * Get the recommended action from an input decision node.
     * @param {const dnode<ST,AC> &} v; input decision node
     * @return Return the recommended action at the input decision node.
     */
    AC recommended_action(const dnode<ST,AC> &v) {
        return v.children.at(argmax_value(v))->a;
    }

    void test(dnode<ST,AC> &v) const {
        std::cout << "nb children = " << v.children.size() << std::endl;
        for(auto &c : v.children) {
            std::cout << "nb children = " << c->children.size() << std::endl;
        }
    }

    /**
     * @brief UCT policy operator
     *
     * Policy operator for the undertaken action at given ST.
     * @param {const ST &} s; current state of the agent
     * @return Return the undertaken action at s.
     */
    AC operator()(const ST &s) {
        dnode<ST,AC> root(s,model.get_action_space(s),nullptr);
        build_uct_tree(root);
        test(root);
        return recommended_action(root);
    }

    /**
     * @brief Process reward
     *
     * Process the resulting reward from transition (s,a,s_p)
     * @param {ST &} s; state
     * @param {AC &} a; action
     * @param {ST &} s_p; next state
     */
    void process_reward(
        const ST & s,
        const AC & a,
        const ST & s_p)
    {
        (void) s;
        (void) a;
        (void) s_p;
        /* No reward processing for UCT policy */
    }

    /**
     * @brief Get backup
     *
     * Get the backed-up values.
     * @return Return a vector containing the values to be saved.
     */
    std::vector<double> get_backup() const {
        return std::vector<double>{(double)global_counter};
    }
};

#endif // UCT_HPP_
