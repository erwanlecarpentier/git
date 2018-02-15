#ifndef L2FSIM_BEELER_GLIDER_COMMAND_HPP_
#define L2FSIM_BEELER_GLIDER_COMMAND_HPP_

#include <command.hpp>

namespace L2Fsim {

/**
 * @brief Beeler's glider command
 * @version 1.0
 * @since 1.0
 */
class beeler_glider_command : public command {
public:
    double dalpha; ///< Variation of angle of attack
    double dbeta; ///< Variation of sideslip angle
    double dsigma; ///< Variation of bank angle

    /** @brief Constructor */
    beeler_glider_command(double _dalpha=0., double _dbeta=0., double _dsigma=0.) :
        dalpha(_dalpha),
        dbeta(_dbeta),
        dsigma(_dsigma)
    {}

    /**
     * @brief Set command
     *
     * Copy the values of the input command.
     * @param {command &} _u; copied command
     */
    void set_command(command &_u) override {
        beeler_glider_command &u = dynamic_cast <beeler_glider_command &> (_u);
        dalpha = u.dalpha;
        dbeta = u.dbeta;
        dsigma = u.dsigma;
    }

    /**
     * @brief Set to neutral
     *
     * Set the command values to neutral values ie zero.
     */
    void set_to_neutral() override {
        dalpha = dbeta = dsigma = 0.;
    }

    /**
     * @brief Equality comparison
     *
     * Compare this command with the given command.
     * @param {const beeler_glider_command &} a; the compared command
     * @return Return true if this action is equal to the argument
     */
    bool equals(const beeler_glider_command &a) {
        if(are_equal(dalpha,a.dalpha)
        && are_equal(dbeta,a.dbeta)
        && are_equal(dsigma,a.dsigma)) {
            return true;
        } else {
            return false;
        }
    }
};

}

#endif
